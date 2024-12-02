import math
from isaacgym import gymapi
from isaacgym import gymutil

# Inicializar o Isaac Gym
gym = gymapi.acquire_gym()

# Configurar parâmetros da simulação
sim_params = gymapi.SimParams()
sim_params.substeps = 2
sim_params.dt = 1.0 / 60.0

sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)
if sim is None:
    print("*** Falha ao criar a simulação")
    quit()

# Criar o visualizador
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise ValueError('*** Falha ao criar o visualizador')

# Adicionar o plano do chão
plane_params = gymapi.PlaneParams()
gym.add_ground(sim, plane_params)

# Definir a grade do ambiente
num_envs = 1
spacing = 1.0
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, 0.0, spacing)

# Criar o ambiente
env = gym.create_env(sim, env_lower, env_upper, num_envs)

# Carregar o modelo do robô 'guaraci'
asset_root = "./"
guaraci_asset_file = "urdf/guaraci.urdf"

guaraci_options = gymapi.AssetOptions()
guaraci_options.fix_base_link = False
guaraci_asset = gym.load_asset(sim, asset_root, guaraci_asset_file, guaraci_options)

# Definir a pose inicial do robô 'guaraci'
guaraci_pose = gymapi.Transform()
guaraci_pose.p = gymapi.Vec3(0.0, 0.15, 0.0)  # Ajuste a altura para acima do chão
angle_rad = math.radians(0)  # 90 graus em torno do eixo Z
guaraci_pose.r = gymapi.Quat.from_euler_zyx(angle_rad, 0, 0)

# Adicionar o robô 'guaraci' ao ambiente
robot_handle = gym.create_actor(env, guaraci_asset, guaraci_pose, 'guaraci', 0, 1)

# Carregar o modelo 'stairs_ramp'
# stairs_asset_file = "urdf/guaraci/urdf/stairs_ramp.urdf"

# stairs_options = gymapi.AssetOptions()
# stairs_options.fix_base_link = True  # A rampa será fixa no chão
# stairs_asset = gym.load_asset(sim, asset_root, stairs_asset_file, stairs_options)

# # Definir a pose inicial da rampa
# stairs_pose = gymapi.Transform()
# stairs_pose.p = gymapi.Vec3(5.0, 0.0, 0.0)  # Ajuste a posição para onde você deseja colocar a rampa

# # Adicionar a rampa ao ambiente
# stairs_handle = gym.create_actor(env, stairs_asset, stairs_pose, 'stairs_ramp', 0, 1)

# # Carregar o modelo 'silo'
# silo_asset_file = "urdf/guaraci/urdf/silo.urdf"

# silo_options = gymapi.AssetOptions()
# silo_options.fix_base_link = True  # A rampa será fixa no chão
# silo_asset = gym.load_asset(sim, asset_root, silo_asset_file, silo_options)

# # Definir a pose inicial da rampa
# silo_pose = gymapi.Transform()
# silo_pose.p = gymapi.Vec3(- 8.0, 0.0, -8.0)  # Ajuste a posição para onde você deseja colocar a rampa

# # Adicionar a rampa ao ambiente
# silo_handle = gym.create_actor(env, silo_asset, silo_pose, 'silo', 0, 1)

# # Carregar o modelo 'silo'
# torre_asset_file = "urdf/guaraci/urdf/torre.urdf"

# torre_options = gymapi.AssetOptions()
# torre_options.fix_base_link = True  # A rampa será fixa no chão
# torre_asset = gym.load_asset(sim, asset_root, torre_asset_file, torre_options)

# # Definir a pose inicial da rampa
# torre_pose = gymapi.Transform()
# torre_pose.p = gymapi.Vec3(8.0, 0.0, -8.0)  # Ajuste a posição para onde você deseja colocar a rampa

# # Adicionar a rampa ao ambiente
# torre_handle = gym.create_actor(env, torre_asset, torre_pose, 'torre', 0, 1)

# # Carregar o modelo 'silo'
# turbina_asset_file = "urdf/guaraci/urdf/turbina.urdf"

# turbina_options = gymapi.AssetOptions()
# turbina_options.fix_base_link = True  # A rampa será fixa no chão
# turbina_asset = gym.load_asset(sim, asset_root, turbina_asset_file, turbina_options)

# # Definir a pose inicial da rampa
# turbina_pose = gymapi.Transform()
# turbina_pose.p = gymapi.Vec3(-8.0, 0.0, 8.0)  # Ajuste a posição para onde você deseja colocar a rampa

# # Adicionar a rampa ao ambiente
# turbina_handle = gym.create_actor(env, turbina_asset, turbina_pose, 'turbina', 0, 1)

# Configurar a câmera
cam_pos = gymapi.Vec3(5, 5, 5)
cam_target = gymapi.Vec3(0, 0, 1)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

# Desenhar o visualizador com os modelos carregados
gym.draw_viewer(viewer, sim, True)

# Loop da simulação
while not gym.query_viewer_has_closed(viewer):
    gym.simulate(sim)
    gym.fetch_results(sim, True)
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)
    gym.sync_frame_time(sim)

print('Simulação finalizada')

# Limpar a simulação
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
