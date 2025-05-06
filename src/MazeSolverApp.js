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
  const [speed, setSpeed] = useState(50); // milliseconds delay
  const [isRunning, setIsRunning] = useState(false);
  const [exploredNodes, setExploredNodes] = useState([]);
  const [path, setPath] = useState([]);
  const [stats, setStats] = useState({ pathLength: 0, nodesExplored: 0, time: 0 });
  const [editMode, setEditMode] = useState(null); // null, 'wall', 'start', 'goal'
  
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
    return true; // Return a value to ensure async chaining works properly
  };
  
  const solveMaze = async () => {
    if (isRunning) return;
    
    setIsRunning(true);
    resetVisualization();
    
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
          result = { path: [], exploredNodes: [] };
      }
      
      const endTime = performance.now();
      
      setPath(result.path);
      setExploredNodes(result.exploredNodes);
      setStats({
        pathLength: result.path.length,
        nodesExplored: result.exploredNodes.length,
        time: (endTime - startTime) / 1000
      });
    } catch (error) {
      console.error("Error solving maze:", error);
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
    <div className="flex flex-col items-center p-4 gap-4">
      <h1 className="text-2xl font-bold">Interactive Maze Solver</h1>
      
      <div className="flex flex-wrap gap-4 mb-4">
        <div>
          <label className="mr-2">Algorithm:</label>
          <select 
            value={algorithm} 
            onChange={(e) => setAlgorithm(e.target.value)}
            disabled={isRunning}
            className="border p-1 rounded"
          >
            <option value="astar">A* Search</option>
            <option value="bfs">Breadth-First Search</option>
            <option value="dfs">Depth-First Search</option>
          </select>
        </div>
        
        <div>
          <label className="mr-2">Speed:</label>
          <select 
            value={speed} 
            onChange={(e) => setSpeed(Number(e.target.value))}
            disabled={isRunning}
            className="border p-1 rounded"
          >
            <option value="10">Very Fast</option>
            <option value="50">Fast</option>
            <option value="100">Medium</option>
            <option value="300">Slow</option>
            <option value="500">Very Slow</option>
          </select>
        </div>
      </div>
      
      <div className="flex flex-wrap gap-2 mb-4">
        <button 
          onClick={solveMaze}
          disabled={isRunning}
          className="bg-blue-500 text-white px-4 py-2 rounded disabled:bg-blue-300"
        >
          {isRunning ? 'Solving...' : 'Solve Maze'}
        </button>
        
        <button 
          onClick={clearPath}
          disabled={isRunning}
          className="bg-yellow-500 text-white px-4 py-2 rounded disabled:bg-yellow-300"
        >
          Clear Path
        </button>
        
        <button 
          onClick={resetMaze}
          disabled={isRunning}
          className="bg-red-500 text-white px-4 py-2 rounded disabled:bg-red-300"
        >
          Reset Maze
        </button>
      </div>
      
      <div className="flex flex-wrap gap-4 mb-4">
        <button 
          onClick={() => setEditMode(editMode === 'wall' ? null : 'wall')}
          disabled={isRunning}
          className={`px-4 py-2 rounded ${editMode === 'wall' ? 'bg-gray-800 text-white' : 'bg-gray-200'}`}
        >
          Edit Walls
        </button>
        
        <button 
          onClick={() => setEditMode(editMode === 'start' ? null : 'start')}
          disabled={isRunning}
          className={`px-4 py-2 rounded ${editMode === 'start' ? 'bg-green-500 text-white' : 'bg-gray-200'}`}
        >
          Set Start
        </button>
        
        <button 
          onClick={() => setEditMode(editMode === 'goal' ? null : 'goal')}
          disabled={isRunning}
          className={`px-4 py-2 rounded ${editMode === 'goal' ? 'bg-blue-500 text-white' : 'bg-gray-200'}`}
        >
          Set Goal
        </button>
      </div>
      
      <div className="mb-4 border rounded p-2 bg-gray-50 w-full max-w-xl">
        <div className="font-bold">Statistics:</div>
        <div>Algorithm: {algorithm === 'astar' ? 'A*' : algorithm === 'bfs' ? 'BFS' : 'DFS'}</div>
        <div>Path Length: {stats.pathLength}</div>
        <div>Nodes Explored: {stats.nodesExplored}</div>
        <div>Time: {stats.time.toFixed(3)} seconds</div>
      </div>
      
      <div className="border border-gray-300 inline-block">
        {maze.map((row, rowIndex) => (
          <div key={rowIndex} className="flex">
            {row.map((cell, colIndex) => (
              <div
                key={`${rowIndex}-${colIndex}`}
                className={`w-8 h-8 border border-gray-200 ${getCellColor(rowIndex, colIndex)} cursor-pointer flex items-center justify-center`}
                onClick={() => handleCellClick(rowIndex, colIndex)}
              >
                {rowIndex === start[0] && colIndex === start[1] && 'S'}
                {rowIndex === goal[0] && colIndex === goal[1] && 'G'}
              </div>
            ))}
          </div>
        ))}
      </div>
      
      <div className="mt-4 text-sm text-gray-600">
        <div className="flex items-center"><span className="inline-block w-4 h-4 bg-white border mr-2"></span> Open Path</div>
        <div className="flex items-center"><span className="inline-block w-4 h-4 bg-black mr-2"></span> Wall</div>
        <div className="flex items-center"><span className="inline-block w-4 h-4 bg-green-500 mr-2"></span> Start Position (S)</div>
        <div className="flex items-center"><span className="inline-block w-4 h-4 bg-blue-500 mr-2"></span> Goal Position (G)</div>
        <div className="flex items-center"><span className="inline-block w-4 h-4 bg-gray-300 mr-2"></span> Explored Node</div>
        <div className="flex items-center"><span className="inline-block w-4 h-4 bg-yellow-400 mr-2"></span> Solution Path</div>
      </div>
    </div>
  );
}