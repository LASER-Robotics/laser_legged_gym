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

from legged_gym.envs import GuaraciRoughCfg, GuaraciRoughCfgPPO

class GuaraciFlatCfg( GuaraciRoughCfg ):
    class env( GuaraciRoughCfg.env ):
        num_observations = 66
  
    class terrain( GuaraciRoughCfg.terrain ):
        mesh_type = 'plane'
        measure_heights = False
  
    class asset( GuaraciRoughCfg.asset ):
        pass

    class rewards( GuaraciRoughCfg.rewards ):
        max_contact_force = 650.
        class scales ( GuaraciRoughCfg.rewards.scales ):
            orientation = -5.0
            torques = -0.000025
            feet_air_time = 2.
            # feet_contact_forces = -0.01
    
    class commands( GuaraciRoughCfg.commands ):
        heading_command = False
        resampling_time = 4.

    class domain_rand( GuaraciRoughCfg.domain_rand ):
        friction_range = [0., 1.5] # on ground planes the friction combination mode is averaging, i.e total friction = (foot_friction + 1.)/2.

class GuaraciFlatCfgPPO( GuaraciRoughCfgPPO ):
    class policy( GuaraciRoughCfgPPO.policy ):
        pass
        # actor_hidden_dims = [128, 64, 32]
        # critic_hidden_dims = [128, 64, 32]
        # activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid

    # class algorithm( GuaraciRoughCfgPPO.algorithm):
    #     entropy_coef = 0.01

    class runner ( GuaraciRoughCfgPPO.runner):
        run_name = ''
        experiment_name = 'flat_guaraci'
        load_run = -1
        max_iterations = 300
