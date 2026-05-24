#include "sim_mujoco.hpp"

RL_Sim* RL_Sim::instance = nullptr;

static const char* kJointNames[] = {
    "FL_hip", "FL_thigh", "FL_knee",
    "FR_hip", "FR_thigh", "FR_knee",
    "RL_hip", "RL_thigh", "RL_knee",
    "RR_hip", "RR_thigh", "RR_knee"
};

RL_Sim::RL_Sim(const std::string& model_xml, const std::string& config_yaml, const std::string& policy_dir_in)
{
    instance = this;
    this->ang_vel_axis = "body";

    std::cout << LOGGER::INFO << "[MuJoCo] Launching..." << std::endl;
    std::cout << LOGGER::INFO << "[MuJoCo] Version: " << mj_versionString() << std::endl;
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    scanPluginLibraries();

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, false);

    std::thread physicsthreadhandle(&PhysicsThread, sim.get(), model_xml.c_str());
    physicsthreadhandle.detach();

    while (1)
    {
        if (d)
        {
            std::cout << LOGGER::INFO << "[MuJoCo] Data prepared" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    this->mj_model = m;
    this->mj_data = d;
    this->SetupSysJoystick("/dev/input/js0", 16);

    this->ReadYaml(config_yaml);
    this->LoadGoalSites();

    this->policy_dir = policy_dir_in;
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    auto fsm_ptr = std::make_shared<FSM>();
    fsm_ptr->AddState(std::make_shared<dog_v2_fsm::RLFSMStatePassive>(this));
    fsm_ptr->AddState(std::make_shared<dog_v2_fsm::RLFSMStateGetUp>(this));
    fsm_ptr->AddState(std::make_shared<dog_v2_fsm::RLFSMStateGetDown>(this));
    fsm_ptr->AddState(std::make_shared<dog_v2_fsm::RLFSMStateRLLocomotion>(this));
    fsm_ptr->SetInitialState("RLFSMStatePassive");
    this->fsm = *fsm_ptr;
    this->fsm.RequestStateChange("RLFSMStateGetUp");

    std::string policies_root = std::filesystem::path(this->policy_dir).parent_path().string();
    this->ScanPolicyConfigs(policies_root);

    std::cout << LOGGER::INFO << "Available policies:" << std::endl;
    for (size_t i = 0; i < this->policy_config_dirs.size(); ++i)
    {
        std::cout << "  [" << (i + 1) << "] " << this->policy_config_dirs[i] << std::endl;
    }

    std::string current_policy_name = std::filesystem::path(this->policy_dir).filename().string();
    this->current_config_idx = 0;
    for (size_t i = 0; i < this->policy_config_dirs.size(); ++i)
    {
        if (this->policy_config_dirs[i] == current_policy_name)
        {
            this->current_config_idx = i;
            break;
        }
    }
    this->config_name = this->policy_config_dirs[this->current_config_idx];

    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

    this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Sim::GetSysJoystick, this));
    this->loop_joystick->start();

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;
    std::cout << LOGGER::INFO << "Keys: 0=GetUp  9=GetDown  P=Passive  W/S/A/D/Q/E=Move  Space=Stop  R=Reset  L=Record  Enter=Pause  1-9=Switch policy" << std::endl;

    sim->RenderLoop();
}

RL_Sim::~RL_Sim()
{
    instance = nullptr;
    this->loop_keyboard->shutdown();
    this->loop_joystick->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    if (record_file.is_open())
        record_file.close();
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::GetState(RobotState<float> *state)
{
    if (mj_data)
    {
        state->base_pos[0] = mj_data->qpos[0];
        state->base_pos[1] = mj_data->qpos[1];
        state->base_pos[2] = mj_data->qpos[2];

        state->imu.quaternion[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 0];
        state->imu.quaternion[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 1];
        state->imu.quaternion[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 2];
        state->imu.quaternion[3] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 3];

        state->imu.gyroscope[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 4];
        state->imu.gyroscope[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 5];
        state->imu.gyroscope[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 6];

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]];
            state->motor_state.dq[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")];
            state->motor_state.tau_est[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + 2 * this->params.Get<int>("num_of_dofs")];
        }
    }
}

void RL_Sim::LoadGoalSites()
{
    if (!this->params.Has("goal_site_names"))
    {
        return;
    }

    mj_forward(this->mj_model, this->mj_data);

    std::vector<std::vector<float>> goals;
    for (const auto& name : this->params.Get<std::vector<std::string>>("goal_site_names"))
    {
        int site_id = mj_name2id(this->mj_model, mjOBJ_SITE, name.c_str());
        if (site_id < 0)
        {
            std::cout << LOGGER::WARNING << "Goal site not found in XML: " << name << std::endl;
            continue;
        }
        goals.push_back({
            static_cast<float>(this->mj_data->site_xpos[3 * site_id + 0]),
            static_cast<float>(this->mj_data->site_xpos[3 * site_id + 1]),
            static_cast<float>(this->mj_data->site_xpos[3 * site_id + 2])
        });
    }
    this->SetGoalPositions(goals);
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    if (mj_data)
    {
        static int print_count = 0;
        if (++print_count % 50 == 0)
        {
            printf("\033[2J\033[H");
            printf("Status: %s  |  Speed: %.2f  |  Steps: %d  |  Policy: [%d] %s\n",
                   this->recording ? "RECORDING" : "idle",
                   this->control.fixed_speed,
                   this->record_step,
                   this->current_config_idx + 1,
                   this->config_name.c_str());
            printf("\n");
            printf("Keys: 0=GetUp  9=GetDown  P=Passive  W/S/A/D/Q/E=Move  Space=Stop  R=Reset  L=Record  Enter=Pause\n");
            printf("Policies:");
            for (size_t i = 0; i < this->policy_config_dirs.size(); ++i)
            {
                printf(" %zu=%s", i + 1, this->policy_config_dirs[i].c_str());
            }
            printf("\n\n");
            printf("%-10s %10s %10s %10s %10s\n", "Joint", "TargetPos", "CurrentPos", "TargetTorque", "Kp");
            printf("--------------------------------------------------------------------------------\n");
        }
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            int joint_idx = this->params.Get<std::vector<int>>("joint_mapping")[i];
            float target_pos = command->motor_command.q[i];
            float current_pos = mj_data->sensordata[joint_idx];
            float target_torque =
                command->motor_command.tau[i] +
                command->motor_command.kp[i] * (target_pos - current_pos) +
                command->motor_command.kd[i] * (command->motor_command.dq[i] - mj_data->sensordata[joint_idx + this->params.Get<int>("num_of_dofs")]);

            mj_data->ctrl[joint_idx] = target_torque;

            if (print_count % 50 == 0)
            {
                printf("%-10s %10.4f %10.4f %10.4f %10.4f\n",
                       kJointNames[i], target_pos, current_pos, target_torque, command->motor_command.kp[i]);
            }
        }
    }
}

void RL_Sim::RobotControl()
{
    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

    this->GetState(&this->robot_state);

    // Check for policy config switch keys (intercept before FSM processes them)
    int new_config_idx = -1;
    switch (this->control.current_keyboard)
    {
    case Input::Keyboard::Num1: new_config_idx = 0; break;
    case Input::Keyboard::Num2: new_config_idx = 1; break;
    case Input::Keyboard::Num3: new_config_idx = 2; break;
    case Input::Keyboard::Num4: new_config_idx = 3; break;
    case Input::Keyboard::Num5: new_config_idx = 4; break;
    case Input::Keyboard::Num6: new_config_idx = 5; break;
    case Input::Keyboard::Num7: new_config_idx = 6; break;
    case Input::Keyboard::Num8: new_config_idx = 7; break;
    case Input::Keyboard::Num9: new_config_idx = 8; break;
    default: break;
    }
    if (new_config_idx >= 0 && new_config_idx < (int)this->policy_config_dirs.size())
    {
        this->SwitchPolicyConfig(new_config_idx);
        this->control.current_keyboard = Input::Keyboard::None;
        this->control.last_keyboard = Input::Keyboard::None;
    }

    this->StateController(&this->robot_state, &this->robot_command);

    if (this->control.current_keyboard == Input::Keyboard::R)
    {
        if (this->mj_model && this->mj_data)
        {
            mj_resetData(this->mj_model, this->mj_data);
            mj_forward(this->mj_model, this->mj_data);
        }
    }

    if (this->control.current_keyboard == Input::Keyboard::L)
    {
        this->recording = !this->recording;
        if (this->recording)
        {
            auto now = std::chrono::system_clock::now();
            auto tt = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << "records/record_" << std::put_time(std::localtime(&tt), "%Y%m%d_%H%M%S") << ".csv";

            auto dir = std::filesystem::path("records");
            if (!std::filesystem::exists(dir))
                std::filesystem::create_directory(dir);

            record_file.open(ss.str());
            int n = this->params.Get<int>("num_of_dofs");
            record_file << "step";
            for (int i = 0; i < n; ++i) record_file << ",dof_pos_" << i;
            for (int i = 0; i < n; ++i) record_file << ",target_dof_pos_" << i;
            record_file << "\n";

            record_step = 0;
            std::cout << LOGGER::INFO << "Recording started: " << ss.str() << std::endl;
        }
        else
        {
            if (record_file.is_open())
                record_file.close();
            std::cout << LOGGER::INFO << "Recording stopped (" << record_step << " steps)" << std::endl;
        }
    }

    if (this->recording && record_file.is_open())
    {
        int n = this->params.Get<int>("num_of_dofs");
        record_file << record_step;
        for (int i = 0; i < n; ++i) record_file << "," << this->robot_state.motor_state.q[i];
        for (int i = 0; i < n; ++i) record_file << "," << this->robot_command.motor_command.q[i];
        record_file << "\n";
        record_step++;
    }

    if (this->control.current_keyboard == Input::Keyboard::Enter)
    {
        if (simulation_running)
        {
            sim->run = 0;
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            sim->run = 1;
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
    }

    this->control.ClearInput();
    this->SetCommand(&this->robot_command);
}

void RL_Sim::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<Joystick>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
    }
    this->sys_js_max_value = (1 << (bits - 1));
}

void RL_Sim::GetSysJoystick()
{
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }

    if (!this->sys_js) return;

    while (this->sys_js->sample(&this->sys_js_event))
    {
        if (this->sys_js_event.isButton())
        {
            this->sys_js_button[this->sys_js_event.number].update(this->sys_js_event.value);
        }
        else if (this->sys_js_event.isAxis())
        {
            double normalized = double(this->sys_js_event.value) / this->sys_js_max_value;
            if (std::abs(normalized) < this->axis_deadzone)
                this->sys_js_axis[this->sys_js_event.number] = 0;
            else
                this->sys_js_axis[this->sys_js_event.number] = this->sys_js_event.value;
        }
    }

    float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        this->control.x = ly;
        this->control.y = lx;
        this->control.yaw = rx;
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->sys_js_active = false;
    }
}

void RL_Sim::ScanPolicyConfigs(const std::string& policies_root)
{
    policy_config_dirs.clear();
    if (!std::filesystem::exists(policies_root)) return;
    for (const auto& entry : std::filesystem::directory_iterator(policies_root))
    {
        if (entry.is_directory())
        {
            std::string config_path = entry.path().string() + "/config.yaml";
            if (std::filesystem::exists(config_path))
            {
                policy_config_dirs.push_back(entry.path().filename().string());
            }
        }
    }
    std::sort(policy_config_dirs.begin(), policy_config_dirs.end());
}

void RL_Sim::SwitchPolicyConfig(int index)
{
    if (index < 0 || index >= (int)policy_config_dirs.size())
        return;

    if (index == this->current_config_idx && this->rl_init_done)
        return;

    if (index != this->current_config_idx)
    {
        std::string policies_root = std::filesystem::path(this->policy_dir).parent_path().string();
        this->policy_dir = policies_root + "/" + policy_config_dirs[index];
        this->config_name = policy_config_dirs[index];
        this->current_config_idx = index;

        std::string config_yaml = this->policy_dir + "/config.yaml";
        this->ReadYaml(config_yaml);

        if (this->mj_model && this->mj_data)
        {
            mj_resetData(this->mj_model, this->mj_data);
            mj_forward(this->mj_model, this->mj_data);
        }

        this->rl_init_done = false;
        std::cout << std::endl << LOGGER::INFO << "Switched to policy: [" << (index + 1) << "] " << policy_config_dirs[index] << std::endl;
    }

    this->pending_rl_switch = true;
    this->fsm.RequestStateChange("RLFSMStateGetUp");
}

void RL_Sim::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;

        this->obs.last_actions = this->obs.actions;
        this->obs.actions.clear();
        this->obs.actions.resize(this->params.Get<int>("num_of_dofs"), 0.0f);

        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        {
            auto cmd3 = this->ComputeGoalCommand(this->robot_state.base_pos, this->robot_state.imu.quaternion);
            auto defaults = this->params.Get<std::vector<float>>("commands_default");
            this->obs.commands.resize(std::max(defaults.size(), size_t(3)));
            this->obs.commands[0] = cmd3[0];
            this->obs.commands[1] = cmd3[1];
            this->obs.commands[2] = cmd3[2];
            for (size_t i = 3; i < this->obs.commands.size(); ++i)
                this->obs.commands[i] = defaults[i];
        }

        if (this->params.Has("commands_clip_lower") && this->params.Has("commands_clip_upper"))
        {
            auto clip_lower = this->params.Get<std::vector<float>>("commands_clip_lower");
            auto clip_upper = this->params.Get<std::vector<float>>("commands_clip_upper");
            if (this->obs.commands.size() == clip_lower.size())
            {
                this->obs.commands = clamp(this->obs.commands, clip_lower, clip_upper);
            }
        }

        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.lin_vel = this->GetLinVel();

        this->UpdateClockInputs();

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
            output_dof_pos_queue.push(this->output_dof_pos);
        if (!this->output_dof_vel.empty())
            output_dof_vel_queue.push(this->output_dof_vel);
        if (!this->output_dof_tau.empty())
            output_dof_tau_queue.push(this->output_dof_tau);
    }
}

std::vector<float> RL_Sim::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", exiting..." << std::endl;
    if (RL_Sim::instance && RL_Sim::instance->sim)
    {
        RL_Sim::instance->sim->exitrequest.store(1);
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    std::string exe_dir = std::filesystem::canonical(std::filesystem::path(argv[0]).parent_path()).string();
    std::string base_dir = std::filesystem::canonical(exe_dir + "/../..").string();

    std::string model_xml = base_dir + "/models/cross_stairs.xml";
    std::string config_yaml = base_dir + "/policy/cts_onnx/config.yaml";

    if (argc >= 2)
    {
        if (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")
        {
            std::cout << "Usage: " << argv[0] << " [policy_config_path]" << std::endl;
            std::cout << "  Default: " << config_yaml << std::endl;
            std::cout << "  Example: " << argv[0] << " policy/wtw/config.yaml" << std::endl;
            return 0;
        }
        config_yaml = argv[1];
        if (config_yaml.find('/') == std::string::npos)
        {
            config_yaml = base_dir + "/policy/" + config_yaml + "/config.yaml";
        }
    }

    std::string policy_dir = std::filesystem::path(config_yaml).parent_path().string();
    try
    {
        YAML::Node config = YAML::LoadFile(config_yaml);
        if (config["params"] && config["params"]["model_xml"])
        {
            std::string config_model_xml = config["params"]["model_xml"].as<std::string>();
            if (config_model_xml.find('/') == 0)
            {
                model_xml = config_model_xml;
            }
            else
            {
                model_xml = base_dir + "/" + config_model_xml;
            }
        }
    }
    catch (const YAML::Exception& e)
    {
        std::cout << LOGGER::WARNING << "Could not read model_xml from config: " << e.what() << std::endl;
    }

    std::cout << LOGGER::INFO << "Model XML: " << model_xml << std::endl;
    std::cout << LOGGER::INFO << "Policy config: " << config_yaml << std::endl;
    std::cout << LOGGER::INFO << "Policy dir: " << policy_dir << std::endl;

    RL_Sim rl_sim(model_xml, config_yaml, policy_dir);
    return 0;
}
