const gridElement = document.getElementById('grid');
const gridSize = 250;
let start = null;
let end = null;

// Create a grid of divs
for (let i = 0; i < gridSize * gridSize; i++) {
    const cell = document.createElement('div');
    cell.classList.add('cell');
    cell.dataset.index = i;
    cell.addEventListener('click', () => handleCellClick(cell));
    gridElement.appendChild(cell);
}



function handleCellClick(cell) {
    if (!start) {
        cell.classList.add('start');
        start = cell.dataset.index;
    } else if (!end) {
        cell.classList.add('end');
        end = cell.dataset.index;
    } else {
        cell.classList.add('wall');
    }
}

document.getElementById('visualize').addEventListener('click', () => {
    fetch('/run-python-script')
        .then(response => response.json())
        .then(data => {
            document.getElementById('output').textContent = data.output;
        })
        .catch(error => console.error('Error:', error));
});


