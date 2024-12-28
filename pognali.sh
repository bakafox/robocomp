#!/bin/bash

# Обновляем сборку пакета autorace_core_ROSticks до последней версии
CURRENT_DIR=$(pwd)

cd ~/ros2_ws
colcon build --packages-select autorace_core_ROSticks
if [ $? -ne 0 ]; then exit 1; fi # Останалвиаем скрипт, если сборка не удалась
source ~/ros2_ws/install/setup.bash
cd "$CURRENT_DIR"


# После этого запускаем TMUX и прочие приколюшки внутри него
SESSION_NAME="ROSticks"

if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "Сессия с именем $SESSION_NAME уже существует! Завершение..."
    tmux kill-session -t $SESSION_NAME
fi

tmux new-session -d -s $SESSION_NAME
tmux split-window -h
tmux split-window -h


# Инициализация трассы, робота, bridge-й и пр. (+ до минор)
tmux send-keys -t 0 "ros2 launch robot_bringup autorace_2023.launch.py" C-m && sleep 5

# Наш пакет для управления роботом (+ до минор)
tmux send-keys -t 1 "ros2 launch autorace_core_ROSticks autorace_core.launch.py" C-m #&& sleep 1

# Телепопа для отладки (качать отсюда: https://index.ros.org/p/teleop_twist_keyboard/)
# tmux send-keys -t 1 "ros2 run teleop_twist_keyboard teleop_twist_keyboard" C-m #&& sleep 1

# Запуск хуйни которая у меня не работает блинб :ССС (+ до минор) (+ смартфон vivo)
tmux send-keys -t 2 "ros2 run referee_console mission_autorace_2023_referee" C-m #&& sleep 1

# Horizontal удобнее в обычном терминале, Vertical в боковом терминале в вскоде
tmux select-layout -t $SESSION_NAME:0 even-vertical
# tmux select-layout -t $SESSION_NAME:0 even-horizontal

tmux attach-session -t $SESSION_NAME
