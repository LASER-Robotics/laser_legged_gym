# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class GuaraciRoughCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env ):
        num_envs = 4096
        num_actions = 18
        num_observations = 253

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'trimesh'

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.6] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            "joint_coxa_r1": 0.0,
            "joint_coxa_r2": 0.0,
            "joint_coxa_r3": 0.0,
            "joint_coxa_l1": 0.0,
            "joint_coxa_l2": 0.0,
            "joint_coxa_l3": 0.0,

            "joint_femur_r1": -0.368,
            "joint_femur_r2": -0.368,
            "joint_femur_r3": -0.368,
            "joint_femur_l1": -0.368,
            "joint_femur_l2": -0.368,
            "joint_femur_l3": -0.368,

            "joint_tibia_r1": 2.043,
            "joint_tibia_r2": 2.043,
            "joint_tibia_r3": 2.043,
            "joint_tibia_l1": 2.043,
            "joint_tibia_l2": 2.043,
            "joint_tibia_l3": 2.043,
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        stiffness = {'coxa': 40., 'femur': 40, 'tibia': 40}  # [N*m/rad]
        damping = {'coxa': 3., 'femur': 3., 'tibia': 3.}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        use_actuator_network = False
        # actuator_net_file = "{LEGGED_GYM_ROOT_DIR}/resources/actuator_nets/anydrive_v3_lstm.pt"

    class asset( LeggedRobotCfg.asset ):
        file = "{LEGGED_GYM_ROOT_DIR}/resources/robots/guaraci/urdf/guaraci.urdf"
        name = "guaraci"
        foot_name = "FOOT"
        penalize_contacts_on = ["femur"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter

    class domain_rand( LeggedRobotCfg.domain_rand):
        randomize_base_mass = False
        added_mass_range = [-5., 5.]
  
    class rewards( LeggedRobotCfg.rewards ):
        base_height_target = 0.4
        max_contact_force = 600.
        only_positive_rewards = True
        class scales( LeggedRobotCfg.rewards.scales ):
            base_height = -1.2
            feet_air_time = 4.

class GuaraciRoughCfgPPO( LeggedRobotCfgPPO ):
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_guaraci'
        load_run = -1
