
# Importiamo i moduli necessari da launch e launch_ros
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,        # Per eseguire comandi shell
    RegisterEventHandler, # Per gestire eventi
    IncludeLaunchDescription,  # Per includere altri launch file
    TimerAction          # Per aggiungere delay temporali
)
from launch.event_handlers import OnProcessExit  # Per gestire l'evento di chiusura di un processo
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Per caricare launch file Python
from launch_ros.actions import Node  # Per creare nodi ROS
from launch.substitutions import PathJoinSubstitution  # Per gestire i percorsi
from launch_ros.substitutions import FindPackageShare  # Per trovare i pacchetti ROS
import os
from datetime import datetime  # Per il timestamp dei file bag

def generate_launch_description():
    # Includiamo il launch file dal pacchetto B
    # FindPackageShare trova il percorso del pacchetto
    # PathJoinSubstitution unisce i percorsi in modo sicuro


    # Primo avvio del state_controller
    initial_state_broadcaster_spawner_stopped = Node(
    package="controller_manager",
        executable="spawner",
        arguments=["state_broadcaster", "-c", "/controller_manager"],
        
    )

    # Secondo avvio del nodo controller
    initial_joint_controller_spawner_stopped = Node(
    package="controller_manager",
        executable="spawner",
        arguments=["joint_controller", "-c", "/controller_manager"],
        
    )


    qualisys_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qualisys_driver'), # Nome del pacchetto in cui è il laucnhe da richiamare
                'launch',
                'qualisys.launch.py' # nome del file
            ])
        ])
    )

    # Definiamo il primo nodo del pacchetto A
    # package: nome del pacchetto
    # executable: nome dell'eseguibile Python
    # name: nome del nodo in runtime
   

    # Definiamo il secondo nodo del pacchetto A
    Jumping_fatigue_test = Node(
        package='fatigue_test',
        executable='fatigue_jump_node',
        name='Jumping_node',
        parameters=[{'test_name': 'softleg_jump_' + datetime.now().strftime("%m_%d_%Y_%H_%M_%S")},
                        {'q_home_1': [-0.7, 2.84]}],  # Parametri che vuoi passare al nodo
        output='screen'
    )

    # Creiamo un timestamp per il nome del file bag
    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    mcap_path = f'bags/test_{timestamp}'
    
    # Configuriamo il processo di registrazione
    # -a: registra tutti i topic
    # -o: specifica il percorso di output
    # --format mcap: usa il formato MCAP
    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', mcap_path],
        output='screen'
    )
    # Handler per quando il primo controller termina
    first_controller_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=initial_state_broadcaster_spawner_stopped,
            on_exit=[initial_joint_controller_spawner_stopped]  # Avvia il secondo controller
        )
    )

    # Handler per quando il secondo controller termina
    second_controller_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=initial_joint_controller_spawner_stopped,
            on_exit=[
                qualisys_launch,  # Avvia Qualisys
                TimerAction(      # Dopo 2 secondi...
                    period=4.0,
                    actions=[
                        record_process,  # Avvia la registrazione
                        Jumping_fatigue_test         # E il secondo nodo
                    ]
                )
            ]
        )
    )

    # Handler per la chiusura finale
    final_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=Jumping_fatigue_test,
            on_exit=[
                ExecuteProcess(
                    cmd=['pkill', '-f', 'ros2 bag record'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/Distributor_node', 'shutdown'],
                    output='screen'
                )
            ]
        )
    )
    # Configuriamo cosa succede quando il secondo nodo termina
    # shutdown_handler = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=Jumping_fatigue_test,  # Monitora il secondo nodo
    #         on_exit=[
    #             # Quando termina, uccide il processo di registrazione
    #             ExecuteProcess(
    #                 cmd=['pkill', '-f', 'ros2 bag record'],
    #                 output='screen'
    #             ),
    #             # E spegne il primo nodo
    #             ExecuteProcess(
    #                 cmd=['ros2', 'lifecycle', 'set', '/moteus_node', 'shutdown'],
    #                 output='screen'
    #             ),
    #             # Chiude il nodo Qualisys (killando tutti i processi con quel nome)
    #             ExecuteProcess(
    #                 cmd=['pkill', '-f', 'qualisys_driver'],
    #                 output='screen'
    #             )
    #             # # Alternativa più specifica se conosci il nome esatto del nodo
    #             # ExecuteProcess(
    #             #     cmd=['ros2', 'lifecycle', 'set', '/qualisys', 'shutdown'],
    #             #     output='screen'
    #             # )
    #         ]
    #     )
    # )
    shutdown_handler = RegisterEventHandler(
    OnProcessExit(
        target_action=Jumping_fatigue_test,
        on_exit=[
            ExecuteProcess(
                cmd=['pkill', '-f', 'ros2 bag record'],
                output='screen'
            ),
            # Usa lo shutdown basato su servizio invece del lifecycle
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/moteus_node/shutdown', 'std_srvs/srv/Trigger', '{}'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['pkill', '-f', 'qualisys_driver'],
                output='screen'
            )
        ]
    )
)


    return LaunchDescription([
        initial_state_broadcaster_spawner_stopped,        # Prima avvia il controller iniziale
        first_controller_handler,     # Gestisce la transizione al secondo controller
        second_controller_handler,    # Gestisce l'avvio di Qualisys e altri nodi
        final_shutdown_handler        # Gestisce la chiusura finale
    ])