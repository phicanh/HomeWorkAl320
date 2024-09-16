/**
 * Runs various graph algorithms on a given graph.
 * 
 * @param {string} input - A string representing the graph edges in the format "u-v-w,u-v-w,..."
 * where u and v are node IDs and w is the weight of the edge.
 */
function runGraphAlgorithms() {
    debugger
    const input = document.getElementById('edges').value;
    const edges = input.split(',').map(edge => edge.split('-').map(Number));

    const graph = buildGraph(edges);
    const vertices = new Set(edges.flatMap(edge => [edge[0], edge[1]])).size;

    document.getElementById('dijkstra').innerText = JSON.stringify(dijkstra(graph, vertices, 0));
    document.getElementById('bellmanFord').innerText = JSON.stringify(bellmanFord(graph, vertices, 0));
    document.getElementById('floydWarshall').innerText = JSON.stringify(floydWarshall(graph, vertices));
    document.getElementById('prim').innerText = JSON.stringify(prim(graph, vertices));
    document.getElementById('kruskal').innerText = JSON.stringify(kruskal(graph, vertices));
    document.getElementById('dfs').innerText = JSON.stringify(dfs(graph, 0));
    document.getElementById('bfs').innerText = JSON.stringify(bfs(graph, 0));
}
/**
 * Builds a graph from a list of edges.
 * 
 * @param {number[][]} edges - A list of edges in the format [[u, v, w], [u, v, w], ...]
 * where u and v are node IDs and w is the weight of the edge.
 * @returns {object} - An adjacency list representation of the graph.
 */
function buildGraph(edges) {
    debugger
    const graph = {};
    edges.forEach(([u, v, w]) => {
        if (!graph[u]) graph[u] = [];
        if (!graph[v]) graph[v] = [];
        graph[u].push({ node: v, weight: w });
        graph[v].push({ node: u, weight: w }); // For undirected graph
    });
    return graph;
}
/**
 * Dijkstra's algorithm for finding the shortest path from a source node to all other nodes.
 * 
 * @param {object} graph - An adjacency list representation of the graph.
 * @param {number} vertices - The number of vertices in the graph.
 * @param {number} start - The source node ID.
 * @returns {number[]} - An array of shortest distances from the source node to all other nodes.
 */
function dijkstra(graph, vertices, start) {
    debugger
    const distances = Array(vertices).fill(Infinity);
    distances[start] = 0;

    const pq = new PriorityQueue();
    pq.enqueue(start, 0);

    while (!pq.isEmpty()) {
        const { element: u, priority: distU } = pq.dequeue();

        if (distU > distances[u]) continue;

        graph[u].forEach(({ node: v, weight }) => {
            const distV = distU + weight;
            if (distV < distances[v]) {
                distances[v] = distV;
                pq.enqueue(v, distV);
            }
        });
    }

    return distances;
}
/**
 * Bellman-Ford algorithm for finding the shortest path from a source node to all other nodes.
 * 
 * @param {object} graph - An adjacency list representation of the graph.
 * @param {number} vertices - The number of vertices in the graph.
 * @param {number} start - The source node ID.
 * @returns {number[]} - An array of shortest distances from the source node to all other nodes.
 */
function bellmanFord(graph, vertices, start) {
    debugger
    const distances = Array(vertices).fill(Infinity);
    distances[start] = 0;

    for (let i = 0; i < vertices - 1; i++) {
        for (let u in graph) {
            graph[u].forEach(({ node: v, weight }) => {
                if (distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                }
            });
        }
    }

    // Check for negative weight cycles
    for (let u in graph) {
        graph[u].forEach(({ node: v, weight }) => {
            if (distances[u] + weight < distances[v]) {
                throw new Error("Graph contains a negative weight cycle");
            }
        });
    }

    return distances;
}
/**
 * Floyd-Warshall algorithm for finding the shortest path between all pairs of nodes.
 * 
 * @param {object} graph - An adjacency list representation of the graph.
 * @param {number} vertices - The number of vertices in the graph.
 * @returns {number[][]} - A 2D array of shortest distances between all pairs of nodes.
 */
function floydWarshall(graph, vertices) {
    debugger
    const distances = Array.from({ length: vertices }, () => Array(vertices).fill(Infinity));
    for (let i = 0; i < vertices; i++) distances[i][i] = 0;

    for (let u in graph) {
        graph[u].forEach(({ node: v, weight }) => {
            distances[u][v] = weight;
        });
    }

    for (let k = 0; k < vertices; k++) {
        for (let i = 0; i < vertices; i++) {
            for (let j = 0; j < vertices; j++) {
                if (distances[i][k] + distances[k][j] < distances[i][j]) {
                    distances[i][j] = distances[i][k] + distances[k][j];
                }
            }
        }
    }

    return distances;
}
/**
 * A JavaScript implementation of Prim's algorithm for finding the Minimum Spanning Tree (MST) of a graph.
 * This implementation uses an edge-weighted, undirected graph as input and returns the parent array of the MST.
 * @param {Object} graph - The input graph in the form of an adjacency list.
 * @param {number} vertices - The number of vertices in the graph.
 * @returns {number[]} The parent array of the MST.
 *
 * Example:
 * const graph = {
 *  0: [{node: 1, weight: 4}, {node: 7, weight: 8}],
 *  1: [{node: 0, weight: 4}, {node: 2, weight: 8}, {node: 7, weight: 11}],
 *  2: [{node: 1, weight: 8}, {node: 3, weight: 7}, {node: 8, weight: 2}],
 *  3: [{node: 2, weight: 7}, {node: 4, weight: 9}, {node: 8, weight: 14}],
 *  4: [{node: 3, weight: 9}, {node: 5, weight: 10}, {node: 7, weight: 16}],
 *  5: [{node: 4, weight: 10}, {node: 6, weight: 11}],
 *  6: [{node: 5, weight: 11}, {node: 7, weight: 6}, {node: 8, weight: 7}],
 *  7: [{node: 0, weight: 8}, {node: 1, weight: 11}, {node: 4, weight: 16}, {node: 6, weight: 6}],
 *  8: [{node: 2, weight: 2}, {node: 3, weight: 14}, {node: 6, weight: 7}]
 *};
 *const vertices = 9;
 *const parent = prim(graph, vertices);
 *console.log(parent); // [null, 0, 1, 2, 3, 4, 5, 6, 2]
 */
function prim(graph, vertices) {
    debugger
    const parent = Array(vertices).fill(-1);
    const key = Array(vertices).fill(Infinity);
    const mstSet = Array(vertices).fill(false);
    key[0] = 0;

    const pq = new PriorityQueue();
    pq.enqueue(0, 0);

    while (!pq.isEmpty()) {
        const { element: u } = pq.dequeue();
        mstSet[u] = true;

        graph[u].forEach(({ node: v, weight }) => {
            if (!mstSet[v] && weight < key[v]) {
                parent[v] = u;
                key[v] = weight;
                pq.enqueue(v, key[v]);
            }
        });
    }

    return parent;
}
/**
 * Kruskal's algorithm for finding the Minimum Spanning Tree (MST) of a graph.
 *
 * @param {Object} graph - The graph represented as an adjacency list.
 * @param {number} vertices - The number of vertices in the graph.
 * @returns {Array} The Minimum Spanning Tree (MST) of the graph, represented as an array of edges.
 *
 * Example:
 * ```
 * const graph = {
 *   0: [{ node: 1, weight: 2 }, { node: 2, weight: 3 }],
 *   1: [{ node: 0, weight: 2 }, { node: 2, weight: 1 }, { node: 3, weight: 4 }],
 *   2: [{ node: 0, weight: 3 }, { node: 1, weight: 1 }, { node: 3, weight: 5 }],
 *   3: [{ node: 1, weight: 4 }, { node: 2, weight: 5 }]
 * };
 * const mst = kruskal(graph, 4);
 * console.log(mst); // Output: [[0, 1, 2], [1, 2, 1], [1, 3, 4]]
 * ```
 */
function kruskal(graph, vertices) {
    debugger
    const parent = Array(vertices).fill(null).map((_, i) => i);
    const rank = Array(vertices).fill(0);

    const edges = [];
    for (let u in graph) {
        graph[u].forEach(({ node: v, weight }) => {
            edges.push([u, v, weight]);
        });
    }
    edges.sort((a, b) => a[2] - b[2]);

    const find = (u) => {
        if (parent[u] !== u) parent[u] = find(parent[u]);
        return parent[u];
    };

    const union = (u, v) => {
        const rootU = find(u);
        const rootV = find(v);

        if (rootU !== rootV) {
            if (rank[rootU] > rank[rootV]) {
                parent[rootV] = rootU;
            } else if (rank[rootU] < rank[rootV]) {
                parent[rootU] = rootV;
            } else {
                parent[rootV] = rootU;
                rank[rootU]++;
            }
        }
    };

    const mst = [];
    for (let [u, v, w] of edges) {
        if (find(u) !== find(v)) {
            union(u, v);
            mst.push([u, v, w]);
        }
    }

    return mst;
}

/**
 * Performs a depth-first search (DFS) on a graph.
 *
 * @param {Object} graph - The graph represented as an adjacency list.
 * @param {string} start - The node to start the search from.
 *
 * @returns {Array} - An array of nodes in the order they were visited.
 *
 * @example
 * const graph = {
 *   'A': [{ node: 'B' }, { node: 'C' }],
 *   'B': [{ node: 'A' }, { node: 'D' }, { node: 'E' }],
 *   'C': [{ node: 'A' }, { node: 'F' }],
 *   'D': [{ node: 'B' }],
 *   'E': [{ node: 'B' }, { node: 'F' }],
 *   'F': [{ node: 'C' }, { node: 'E' }]
 * };
 *
 * const result = dfs(graph, 'A');
 * console.log(result); // ['A', 'B', 'D', 'E', 'F', 'C']
 */
function dfs(graph, start) {
    debugger
    const visited = new Set();
    const result = [];

    function visit(node) {
        if (!visited.has(node)) {
            visited.add(node);
            result.push(node);
            graph[node].forEach(({ node: neighbor }) => visit(neighbor));
        }
    }

    visit(start);
    return result;
}

/**
 * Performs a breadth-first search (BFS) on a graph.
 *
 * @param {Object} graph - The graph represented as an adjacency list.
 * @param {string} start - The node to start the search from.
 *
 * @returns {Array} - An array of nodes in the order they were visited.
 *
 * @example
 * const graph = {
 *   'A': [{ node: 'B' }, { node: 'C' }],
 *   'B': [{ node: 'A' }, { node: 'D' }, { node: 'E' }],
 *   'C': [{ node: 'A' }, { node: 'F' }],
 *   'D': [{ node: 'B' }],
 *   'E': [{ node: 'B' }, { node: 'F' }],
 *   'F': [{ node: 'C' }, { node: 'E' }]
 * };
 *
 * const result = bfs(graph, 'A');
 * console.log(result); // ['A', 'B', 'C', 'D', 'E', 'F']
 */
function bfs(graph, start) {
    debugger
    const visited = new Set();
    const queue = [start];
    const result = [];

    while (queue.length > 0) {
        const node = queue.shift();
        if (!visited.has(node)) {
            visited.add(node);
            result.push(node);
            graph[node].forEach(({ node: neighbor }) => {
                if (!visited.has(neighbor)) {
                    queue.push(neighbor);
                }
            });
        }
    }

    return result;
}

/**
 * A priority queue implementation in JavaScript.
 * 
 * A priority queue is a data structure that allows elements to be added and removed based on their priority.
 * Elements with higher priority are removed before elements with lower priority.
 * 
 * @class PriorityQueue
 */
class PriorityQueue {
    /**
     * Creates a new priority queue.
     * 
     * @constructor
     */
    constructor() {
        this.values = [];
    }

    /**
     * Adds an element to the priority queue with the specified priority.
     * 
     * @param {*} element The element to add to the queue.
     * @param {number} priority The priority of the element.
     * @example
     * const queue = new PriorityQueue();
     * queue.enqueue('low priority task', 1);
     * queue.enqueue('high priority task', 5);
     */
    enqueue(element, priority) {
        this.values.push({ element, priority });
        this.sort();
    }

    /**
     * Removes and returns the element with the highest priority from the queue.
     * 
     * @returns {{element: *, priority: number}} The removed element and its priority.
     * @example
     * const queue = new PriorityQueue();
     * queue.enqueue('low priority task', 1);
     * queue.enqueue('high priority task', 5);
     * const removed = queue.dequeue();
     * console.log(removed); // { element: 'high priority task', priority: 5 }
     */
    dequeue() {
        return this.values.shift();
    }

    /**
     * Sorts the elements in the queue based on their priority.
     * 
     * @private
     */
    sort() {
        this.values.sort((a, b) => a.priority - b.priority);
    }

    /**
     * Checks if the queue is empty.
     * 
     * @returns {boolean} True if the queue is empty, false otherwise.
     * @example
     * const queue = new PriorityQueue();
     * console.log(queue.isEmpty()); // true
     * queue.enqueue('task', 1);
     * console.log(queue.isEmpty()); // false
     */
    isEmpty() {
        return this.values.length === 0;
    }
}
