const manualStates = [
    { title: 'idle:', value: 0 },
    { title: 'teleop:', value: 1 },
    { title: 'visual servo:', value: 2 },
    { title: 'return to init:', value: 3 }
];

const autonomousStates = [
    { title: 'initialize:', value: 4 },
    { title: 'visual servo:', value: 5 },
    { title: 'cartesian move:', value: 6 },
    { title: 'harvest:', value: 7 },
    { title: 'move to basket:', value: 8 }
];


// JavaScript for handling the state machines and timers
function createStateCircles(count, containerId, prefix) {
    const container = document.getElementById(containerId);
    container.classList.add('state-machine-container'); // Add a container class

    for (let i = 1; i <= count; i++) {
        const stateCircle = document.createElement('div');
        stateCircle.className = 'state-circle';
        stateCircle.id = `${prefix}-state-${i}`;
        stateCircle.textContent = `${prefix.toUpperCase()}${i}`;
        container.appendChild(stateCircle);

        if (i < count) {
            const arrow = document.createElement('div');
            arrow.className = 'arrow';
            container.appendChild(arrow);
        }
    }
}

function updateStateMachine(state, prefix) {
    for (let i = 1; i <= 8; i++) {
        const stateCircle = document.getElementById(`${prefix}-state-${i}`);
        stateCircle.classList.remove('active');
    }
    const activeState = document.getElementById(`${prefix}-state-${state}`);
    if (activeState) {
        activeState.classList.add('active');
    }
}

// Timer functionality
let timers = [];

function startTimer(timerId) {
    const timerElement = document.getElementById(`timer-${timerId}`);
    let seconds = 0;
    timers[timerId] = setInterval(() => {
        seconds++;
        timerElement.textContent = `Timer ${timerId}: ${new Date(seconds * 1000).toISOString().substr(11, 8)}`;
    }, 1000);
}

function createTimer(timerId) {
    const timerElement = document.createElement('div');
    timerElement.className = 'timer';
    timerElement.id = `timer-${timerId}`;
    timerElement.textContent = `Timer ${timerId}: 00:00:00`;

    const startButton = document.createElement('button');
    startButton.textContent = 'Start';
    startButton.onclick = () => startTimer(timerId);
    timerElement.appendChild(startButton);

    const timersContainer = document.getElementById('timers');
    timersContainer.appendChild(timerElement);
}

// Initialization

document.addEventListener('DOMContentLoaded', () => {
    createStateCircles(8, 'system-state-machine', 'system', 'state-machine-container'); // Add 'state-machine-container' class
    createStateCircles(4, 'amiga-state-machine', 'amiga', 'state-machine-container'); // Add 'state-machine-container' class
    for (let i = 1; i <= 6; i++) {
        createTimer(i);
    }

    const socket = io.connect(location.protocol + '//' + document.domain + ':' + location.port);

    // Listen for state update events
    socket.on('state_update', function (data) {
        updateStateMachine(data.state, 'system');
    });
});

// // Simulated ROS topic subscription callback
// // Replace with actual WebSocket or ROS topic callback to set state
// function simulateRosCallback() {
//     const systemState = Math.floor(Math.random() * 8) + 1; // Random state between 1 and 8
//     const amigaState = Math.floor(Math.random() * 4) + 1;  // Random state between 1 and 4
//     updateStateMachine(systemState, 'system');
//     updateStateMachine(amigaState, 'amiga');
// }

// // Simulate ROS topic update every 5 seconds
// setInterval(simulateRosCallback, 5000);
// Initialize a Socket.IO connection

