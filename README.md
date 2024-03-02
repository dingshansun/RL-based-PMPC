# RL-based-PMPC
Case study codes of the paper ``Adaptive Parameterized Model Predictive Control Based on Reinforcement Learning: A Synthesis Framework"

There are 6 controllers for comparison, each of which corresponds to a .m file:

No-control case --> Main_openloop.m  
ALINEA controller --> ALINEA.m  
Standalone MPC controller --> MPC_RM_VSL.m  
Standalone PMPC controller --> PMPC_RM_VSL.m  
Standalone RL controller --> Ramp_metering_learning.m  
RL-PMPC controller --> PMPC_DQN_RM.m  

Note that Ramp_metering_learning.m and PMPC_DQN_RM.m are the files to start the learning process. After the learning is finished, one implement the learnt agent to validate its control performance.
