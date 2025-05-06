import { useState, useEffect, useRef } from 'react';
import './mazesolver.css';

function reconstructPath(cameFrom, current) {
  const path = [current];
  let currentKey = `${current[0]},${current[1]}`;
  
  while (currentKey in cameFrom && cameFrom[currentKey] !== null) {
    current = cameFrom[currentKey];
    currentKey = `${current[0]},${current[1]}`;
    path.unshift(current); 
    
  }
  
  return path;
}

// Define the MazeSolver class
class MazeSolver {
  constructor(maze) {
    this.maze = maze;
    this.height = maze.length;
    this.width = maze.length > 0 ? maze[0].length : 0;
    this.directions = [[0, 1], [1, 0], [0, -1], [-1, 0]]; // right, down, left, up
  }

  isValid(x, y) {
    return x >= 0 && x < this.height && y >= 0 && y < this.width && this.maze[x][y] === 0;
  }

  manhattanDistance(x1, y1, x2, y2) {
    return Math.abs(x1 - x2) + Math.abs(y1 - y2);
  }

  async aStar(start, goal, updateVisualization, delay) {
    const openSet = [];  // Priority queue for A*
    const closedSet = new Set();  // Set of visited positions
    const gScore = {};  // Cost from start to current position
    gScore[`${start[0]},${start[1]}`] = 0;
    
    const fScore = {}; // f = g + h (cost + heuristic)
    fScore[`${start[0]},${start[1]}`] = this.manhattanDistance(start[0], start[1], goal[0], goal[1]);
    
    const cameFrom = {};  // Parent pointers for reconstructing the path
    
    // Insert start into priority queue
    openSet.push([fScore[`${start[0]},${start[1]}`], start]);
    
    const exploredNodes = [];
    
    while (openSet.length > 0) {
      // Sort the open set by f-score (priority)
      openSet.sort((a, b) => a[0] - b[0]);
      
      // Get the node with lowest f_score
      const current = openSet.shift()[1];
      const currentKey = `${current[0]},${current[1]}`;
      
      // Add to explored nodes for visualization
      exploredNodes.push(current);
      
      // Update visualization
      await updateVisualization(exploredNodes, reconstructPath(cameFrom, current));
      if (delay > 0) {
        await new Promise(resolve => setTimeout(resolve, delay));
      }
      
      // If goal reached, reconstruct and return the path
      if (current[0] === goal[0] && current[1] === goal[1]) {
        const path = reconstructPath(cameFrom, current);
        return { path, exploredNodes };
      }
      
      // Add current node to closed set
      closedSet.add(currentKey);
      
      // Explore neighbors
      for (const [dx, dy] of this.directions) {
        const neighbor = [current[0] + dx, current[1] + dy];
        const neighborKey = `${neighbor[0]},${neighbor[1]}`;
        
        // Skip if not valid or already processed
        if (!this.isValid(neighbor[0], neighbor[1]) || closedSet.has(neighborKey)) {
          continue;
        }
        
        // Calculate tentative g_score for neighbor
        const tentativeG = gScore[currentKey] + 1;
        
        // If new path to neighbor is better
        if (!(neighborKey in gScore) || tentativeG < gScore[neighborKey]) {
          // Update neighbor information
          cameFrom[neighborKey] = current;
          gScore[neighborKey] = tentativeG;
          fScore[neighborKey] = tentativeG + this.manhattanDistance(
            neighbor[0], neighbor[1], goal[0], goal[1]
          );
          
          // Add to open set if not already there
          if (!openSet.some(item => item[1][0] === neighbor[0] && item[1][1] === neighbor[1])) {
            openSet.push([fScore[neighborKey], neighbor]);
          }
        }
      }
    }
    
    // No path found
    return { path: [], exploredNodes };
  }
  
  async breadthFirstSearch(start, goal, updateVisualization, delay) {
    const queue = [start];
    const visited = {};  // Maps positions to their parent
    visited[`${start[0]},${start[1]}`] = null;
    
    const exploredNodes = [];
    
    while (queue.length > 0) {
      const current = queue.shift();
      const currentKey = `${current[0]},${current[1]}`;
      
      exploredNodes.push(current);
      
      // Update visualization
      await updateVisualization(exploredNodes, reconstructPath(visited, current));
      if (delay > 0) {
        await new Promise(resolve => setTimeout(resolve, delay));
      }
      
      if (current[0] === goal[0] && current[1] === goal[1]) {
        // Reconstruct the path
        const path = reconstructPath(visited, current);
        return { path, exploredNodes };
      }
      
      // Explore neighbors
      for (const [dx, dy] of this.directions) {
        const neighbor = [current[0] + dx, current[1] + dy];
        const neighborKey = `${neighbor[0]},${neighbor[1]}`;
        
        if (this.isValid(neighbor[0], neighbor[1]) && !(neighborKey in visited)) {
          queue.push(neighbor);
          visited[neighborKey] = current;
        }
      }
    }
    
    return { path: [], exploredNodes };
  }
  
  async depthFirstSearch(start, goal, updateVisualization, delay) {
    const stack = [start];
    const visited = {};  // Maps positions to their parent
    visited[`${start[0]},${start[1]}`] = null;
    
    const exploredNodes = [];
    
    while (stack.length > 0) {
      const current = stack.pop();
      const currentKey = `${current[0]},${current[1]}`;
      
      exploredNodes.push(current);
      
      // Update visualization
      await updateVisualization(exploredNodes, reconstructPath(visited, current));
      if (delay > 0) {
        await new Promise(resolve => setTimeout(resolve, delay));
      }
      
      if (current[0] === goal[0] && current[1] === goal[1]) {
        // Reconstruct the path
        const path = reconstructPath(visited, current);
        return { path, exploredNodes };
      }
      
      // Explore neighbors in reverse order to match typical DFS behavior
      for (let i = this.directions.length - 1; i >= 0; i--) {
        const [dx, dy] = this.directions[i];
        const neighbor = [current[0] + dx, current[1] + dy];
        const neighborKey = `${neighbor[0]},${neighbor[1]}`;
        
        if (this.isValid(neighbor[0], neighbor[1]) && !(neighborKey in visited)) {
          stack.push(neighbor);
          visited[neighborKey] = current;
        }
      }
    }
    
    return { path: [], exploredNodes };
  }
}

// Sample maze (0 = path, 1 = wall)
const sampleMaze = [
  [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, 1, 0, 1, 1, 1, 1, 1, 1, 0],
  [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
  [0, 1, 1, 1, 1, 0, 1, 0, 1, 0],
  [0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
  [0, 1, 1, 0, 1, 0, 1, 0, 1, 0],
  [0, 0, 1, 0, 1, 0, 1, 0, 1, 0],
  [0, 1, 1, 0, 1, 0, 1, 0, 1, 0],
  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
  [0, 1, 1, 1, 1, 1, 1, 1, 1, 0]
];

// Main component
export default function MazeSolverApp() {
  const [maze, setMaze] = useState(sampleMaze);
  const [start, setStart] = useState([0, 0]);
  const [goal, setGoal] = useState([8, 9]);
  const [algorithm, setAlgorithm] = useState('astar');
  const [speed, setSpeed] = useState(50);
  const [isRunning, setIsRunning] = useState(false);
  const [exploredNodes, setExploredNodes] = useState([]);
  const [path, setPath] = useState([]);
  const [stats, setStats] = useState({ pathLength: 0, nodesExplored: 0, time: 0 });
  const [editMode, setEditMode] = useState(null);
  const [progress, setProgress] = useState(0);
  const [totalCells, setTotalCells] = useState(0);
  
  // Use refs to avoid closure issues with async functions
  const mazeRef = useRef(maze);
  const solverRef = useRef(new MazeSolver(maze));
  
  // Update the solver when maze changes
  useEffect(() => {
    mazeRef.current = maze;
    solverRef.current = new MazeSolver(maze);
  }, [maze]);
  
  const resetVisualization = () => {
    setExploredNodes([]);
    setPath([]);
    setStats({ pathLength: 0, nodesExplored: 0, time: 0 });
  };
  
  const updateVisualization = async (explored, currentPath) => {
    setExploredNodes([...explored]);
    setPath([...currentPath]);
    
    // Update progress
    const total = maze.length * maze[0].length;
    const exploredCount = explored.length;
    const progressPercent = (exploredCount / total) * 100;
    setProgress(Math.min(progressPercent, 100));
    
    return true;
  };
  
  const solveMaze = async () => {
    if (isRunning) return;
    
    setIsRunning(true);
    resetVisualization();
    setProgress(0);
    
    const startTime = performance.now();
    let result;
    
    try {
      switch (algorithm) {
        case 'astar':
          result = await solverRef.current.aStar(start, goal, updateVisualization, speed);
          break;
        case 'bfs':
          result = await solverRef.current.breadthFirstSearch(start, goal, updateVisualization, speed);
          break;
        case 'dfs':
          result = await solverRef.current.depthFirstSearch(start, goal, updateVisualization, speed);
          break;
        default:
          result = await solverRef.current.aStar(start, goal, updateVisualization, speed);
      }
      
      const endTime = performance.now();
      const timeTaken = (endTime - startTime) / 1000;
      
      setStats({
        pathLength: result.path.length,
        nodesExplored: result.exploredNodes.length,
        time: timeTaken.toFixed(2)
      });
      
      setPath(result.path);
      setProgress(100);
    } catch (error) {
      console.error('Error solving maze:', error);
    } finally {
      setIsRunning(false);
    }
  };
  
  const handleCellClick = (row, col) => {
    if (isRunning) return;
    
    const newMaze = [...maze.map(row => [...row])]; // Deep copy
    
    if (editMode === 'wall') {
      // Toggle wall/path (but don't place walls on start or goal)
      if ((row !== start[0] || col !== start[1]) && (row !== goal[0] || col !== goal[1])) {
        newMaze[row][col] = newMaze[row][col] === 0 ? 1 : 0;
        setMaze(newMaze);
      }
    } else if (editMode === 'start') {
      // Don't place start on a wall or goal
      if (newMaze[row][col] === 0 && (row !== goal[0] || col !== goal[1])) {
        setStart([row, col]);
      }
    } else if (editMode === 'goal') {
      // Don't place goal on a wall or start
      if (newMaze[row][col] === 0 && (row !== start[0] || col !== start[1])) {
        setGoal([row, col]);
      }
    }
    
    resetVisualization();
  };
  
  const resetMaze = () => {
    if (isRunning) return;
    setMaze(sampleMaze);
    setStart([0, 0]);
    setGoal([8, 9]);
    resetVisualization();
  };
  
  const clearPath = () => {
    if (isRunning) return;
    resetVisualization();
  };
  
  const getCellColor = (row, col) => {
    // Check if cell is start or goal
    if (row === start[0] && col === start[1]) {
      return 'bg-green-500';
    }
    if (row === goal[0] && col === goal[1]) {
      return 'bg-blue-500';
    }
    
    // Check if cell is wall
    if (maze[row][col] === 1) {
      return 'bg-black';
    }
    
    // Check if cell is part of the path
    const isOnPath = path.some(([r, c]) => r === row && c === col);
    if (isOnPath) {
      return 'bg-yellow-400';
    }
    
    // Check if cell was explored
    const wasExplored = exploredNodes.some(([r, c]) => r === row && c === col);
    if (wasExplored) {
      return 'bg-gray-300';
    }
    
    // Default path color
    return 'bg-white';
  };
  
  return (
    <div className="container">
      <h1>Maze Solver Visualizer</h1>
      
      <div className="controls">
        <div className="control-group">
          <label htmlFor="algorithm">Algorithm:</label>
          <select
            id="algorithm"
            value={algorithm}
            onChange={(e) => setAlgorithm(e.target.value)}
            disabled={isRunning}
          >
            <option value="astar">A* Search</option>
            <option value="bfs">Breadth-First Search</option>
            <option value="dfs">Depth-First Search</option>
          </select>
        </div>
        
        <div className="control-group">
          <label htmlFor="speed">Speed:</label>
          <select
            id="speed"
            value={speed}
            onChange={(e) => setSpeed(Number(e.target.value))}
            disabled={isRunning}
          >
            <option value="0">Instant</option>
            <option value="50">Fast</option>
            <option value="100">Normal</option>
            <option value="200">Slow</option>
          </select>
        </div>
      </div>

      <div className="button-row">
        <button
          className="solve-button"
          onClick={solveMaze}
          disabled={isRunning}
        >
          {isRunning ? 'Solving...' : 'Solve Maze'}
        </button>
        <button
          className="clear-button"
          onClick={clearPath}
          disabled={isRunning}
        >
          Clear Path
        </button>
        <button
          className="reset-button"
          onClick={resetMaze}
          disabled={isRunning}
        >
          Reset Maze
        </button>
      </div>

      {isRunning && (
        <div className="progress-container">
          <div
            className="progress-bar"
            style={{ width: `${progress}%` }}
          />
        </div>
      )}

      <div className="stats">
        <div className="stats-title">Statistics</div>
        <div>Path Length: {stats.pathLength}</div>
        <div>Nodes Explored: {stats.nodesExplored}</div>
        <div>Time Taken: {stats.time}s</div>
      </div>

      <div className="maze-container">
        <div className="maze-grid">
          {maze.map((row, rowIndex) => (
            <div key={rowIndex} className="maze-row">
              {row.map((cell, colIndex) => {
                const isStart = rowIndex === start[0] && colIndex === start[1];
                const isGoal = rowIndex === goal[0] && colIndex === goal[1];
                const isExplored = exploredNodes.some(
                  ([r, c]) => r === rowIndex && c === colIndex
                );
                const isPath = path.some(
                  ([r, c]) => r === rowIndex && c === colIndex
                );
                
                return (
                  <div
                    key={`${rowIndex}-${colIndex}`}
                    className={`cell ${
                      isStart
                        ? 'cell-start'
                        : isGoal
                        ? 'cell-goal'
                        : cell === 1
                        ? 'cell-wall'
                        : isPath
                        ? 'cell-solution'
                        : isExplored
                        ? 'cell-explored'
                        : 'cell-path'
                    }`}
                    onClick={() => handleCellClick(rowIndex, colIndex)}
                  />
                );
              })}
            </div>
          ))}
        </div>
      </div>

      <div className="legend">
        <div className="legend-item">
          <div className="legend-color path-color" />
          <span>Path</span>
        </div>
        <div className="legend-item">
          <div className="legend-color wall-color" />
          <span>Wall</span>
        </div>
        <div className="legend-item">
          <div className="legend-color start-color" />
          <span>Start</span>
        </div>
        <div className="legend-item">
          <div className="legend-color goal-color" />
          <span>Goal</span>
        </div>
        <div className="legend-item">
          <div className="legend-color explored-color" />
          <span>Explored</span>
        </div>
        <div className="legend-item">
          <div className="legend-color solution-color" />
          <span>Solution</span>
        </div>
      </div>
    </div>
  );
}