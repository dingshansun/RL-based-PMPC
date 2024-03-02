state_space=[29 1]; % Continuous states
action_space=0:10; % Discrete actions
obsInfo = rlNumericSpec(state_space);
obsInfo.Name='Weather and density';
actInfo = rlFiniteSetSpec(action_space);
actInfo.Name='Critical density';

criticNet=[featureInputLayer(prod(obsInfo.Dimension),'Normalization','none','Name','observation')
    fullyConnectedLayer(64,'Name','ActorFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(256,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(64,'Name','ActorFC3')
    reluLayer('Name','ActorRelu3')
    fullyConnectedLayer(numel(actInfo.Elements),'Name','ActorFC4')
    ];
critic = rlVectorQValueFunction(criticNet,obsInfo,actInfo);

agent = rlDQNAgent(critic); % Default DQN agent

agent.AgentOptions.UseDoubleDQN = false;
agent.AgentOptions.TargetSmoothFactor = 1e-2;
agent.AgentOptions.TargetUpdateFrequency = 10;
agent.AgentOptions.ExperienceBufferLength = 1e5;
agent.AgentOptions.MiniBatchSize = 512;
agent.AgentOptions.CriticOptimizerOptions.LearnRate = 1e-3;
agent.AgentOptions.CriticOptimizerOptions.GradientThreshold = 1;
agent.AgentOptions.NumStepsToLookAhead=10;
agent.AgentOptions.EpsilonGreedyExploration.EpsilonMin=0.01;
agent.AgentOptions.EpsilonGreedyExploration.EpsilonDecay=0.005;

env = rlFunctionEnv(obsInfo,actInfo,'Ramp_step','Ramp_reset');

trainOpts = rlTrainingOptions;
trainOpts.MaxEpisodes = 2000;
trainOpts.MaxStepsPerEpisode = 120;
trainOpts.SaveAgentCriteria = 'AverageReward';
trainOpts.SaveAgentValue = -7000;

Training = train(agent,env,trainOpts);

% save('Saved_results/DQN_RM_weather23_realmodel.mat');