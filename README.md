# Integrating Intent Understanding and Optimal Behavior Planning for Behavior Tree Generation from Human instructions (IJCAI 2024)

[[Website]](https://dids-ei.github.io/Project/LLM-OBTEA/) [[arXiv]](https://arxiv.org/pdf/2405.07474)

This repository contains the code for our project, which is divided into three main sections:

1. **LLM Folder**: This directory contains the code for the Large Language Models (LLM). It encompasses all the scripts and modules necessary to run and implement the large-scale language model functionalities of our project.

2. **OBTEA Folder**: Here, you will find the code for the Behavior Tree Backward Expansion Algorithm (OBTEA). This section includes the implementation details and the necessary scripts for executing the behavior tree-based algorithms.

3. **RoboWaiter Folder**:  It contains code for deploying our algorithms in a café's digital twin scenario, showcasing the practical application of our research in simulating and enhancing café service operations.

   👉 [Download Simulator](https://drive.google.com/file/d/1ayAQZbPOyQV2W-V_ZdYv6AoqLOg0zvm1/view?usp=sharing)

We are committed to continuously updating and improving this repository, so stay tuned for future enhancements and additions.


✨️ We have uploaded the slim version at `https://github.com/DIDS-EI/OBTEA-demo`. If you want to generate a BT file for a custom task, you can refer to `OBTEA-demo/test_demo/run_demo_task.py`.
1. First, create your own environment under `OBTEA-demo\btpg\envs`, such as `DemoEasy`. The key is to establish the action classes and condition classes in `exec_lib`. Pay attention to the preconditions (`pre`), additions (`add`), deletions (`del`), and their optional parameters for each action.
2. In the main function `run_demo_task.py`, provide the path to `exec_lib` to import `behavior_lib`.
3. Before running the BT algorithm, specify the goal and the current state `cur_cond_set`.
4. To draw the BT, you need the `.btml` file and the imported `behavior_lib`.
We will continue to update and maintain this project, so stay tuned!




## License

This repository is released under the MIT license as found in the [LICENSE](LICENSE) file.
