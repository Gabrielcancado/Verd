from setuptools import setup

# Nome do pacote ROS 2
package_name = 'robot_description'

# Configuração do pacote usando setuptools
setup(
    name=package_name,  # Nome do pacote
    version='0.0.1',  # Versão inicial do pacote
    packages=[],  # Não há pacotes Python, apenas scripts
    data_files=[  # Arquivos adicionais que serão instalados junto com o pacote
        # Registra o pacote no índice de recursos do ROS 2
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Adiciona o arquivo package.xml que descreve o pacote
        ('share/' + package_name, ['package.xml']),
        # Inclui os arquivos de lançamento (launch files) para ROS 2
        ('share/' + package_name + '/launch', ['launch/display.launch.py', 'launch/gazebo.launch.py']),
        # Inclui arquivos de configuração, como RViz e controladores
        ('share/' + package_name + '/config', ['config/link2_inertia.yaml', 'config/controllers.yaml']),
         # Inclui arquivos de configuração do RViz
        ('share/' + package_name + '/rviz2', ['rviz2/display.rviz' ]),
        # Inclui os modelos URDF/Xacro necessários para a simulação do robô
        ('share/' + package_name + '/urdf', [
            'urdf/gazebo.xacro',  # Arquivo de simulação do robô no gazebo
            'urdf/robot_description.urdf',  # Arquivo principal do robô em formato urdf
            'urdf/robot_description.xacro',  #Arquivo principal do robô em formato Xacro
        ]),
        # Adiciona os arquivos de malha (STL) para visualização do robô
        ('share/' + package_name + '/meshes', [
            'meshes/Base.stl', 
            'meshes/Bucha1.stl', 
            'meshes/Engrenagem1.stl', 
            'meshes/Engrenagem2.stl',
            'meshes/Estrutura1.stl',
            'meshes/Estrutura2.stl',
            'meshes/Estrutura3.stl',
            'meshes/FusoDeRosca.stl',
            'meshes/Motor1.stl',
            'meshes/Motor2.stl',
            'meshes/Tampalink2.stl',
            'meshes/TampaLinkP.stl',
        ]),
    ],
   # scripts=[
        #'scripts/kinematics.py',  # Referência ao script direto
    #],

    install_requires=['setuptools'],  # Dependência necessária para instalação
    zip_safe=True,  # Indica que o pacote pode ser distribuído como um arquivo zip
    maintainer='salem',  # Nome do responsável pela manutenção do pacote
    maintainer_email='sabrinaseba18@gmail.com',  # Email de contato do mantenedor
    description='Pacote de simulação do manipulador soldador Fury Forge',  # Descrição do pacote
    #license='MIT',  # Licença do pacote
    tests_require=['pytest'],  # Dependências para rodar testes

    #entry_points={
    #    'console_scripts': [
    #        'kinematics = kinematics:main',  # Referência correta ao script 
    #    ],
    #},
)