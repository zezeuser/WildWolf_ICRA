
基本参数：action_dim = 1*7    state_dim=1*30   if_discrete = Flase(代表连续动作空间)

更改state_dim , action_dim需要在rmaics类里更改,其次在agent 中的 explore_env 函数中的range 和run.Evaluator类中的get_episode_return 函数 中的range更改为对应的action_dim

更改run.explore_before_training函数 rd.randint(-1,2,size=action_dim)   randint取值范围不包含第二个参数














报错：expected scalar type Double but found Float 。解决：更改run.Evaluator类中的get_episode_return 函数中的s_tensor = torch.as_tensor((state),  dtype=torch.float32,device=device) 



