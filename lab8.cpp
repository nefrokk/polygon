#include <iostream>
#include <unordered_map>
#include <vector>
#include <stack>
#include <queue>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cassert>

using namespace std;

// Структура Node для представления узла графа
struct Node {
    double lon; // Долгота
    double lat; // Широта

    // Оператор сравнения для узлов
    bool operator==(const Node& other) const {
        return lon == other.lon && lat == other.lat;
    }

    // Оператор неравенства для узлов
    bool operator!=(const Node& other) const {
        return !(*this == other);
    }

    // Оператор меньше для узлов (для использования в приоритетной очереди)
    bool operator<(const Node& other) const {
        if (lon != other.lon)
            return lon < other.lon;
        return lat < other.lat;
    }
};

// Специализация хеш-функции для структуры Node
namespace std {
    template <>
    struct hash<Node> {
        size_t operator()(const Node& node) const {
            return hash<double>()(node.lon) ^ hash<double>()(node.lat);
        }
    };
}

// Класс Graph для представления графа
class Graph {
public:
    // Метод для парсинга данных графа из строки
    void parseData(const string& data) {
        stringstream ss(data);
        string line;
        while (getline(ss, line)) {
            stringstream lineStream(line);
            double lon1, lat1, lon2, lat2, weight;
            char comma;
            if (lineStream >> lon1 >> comma >> lat1 && comma == ',' && lineStream.get() == ':') {
                Node node1{ lon1, lat1 };
                while (lineStream >> lon2 >> comma >> lat2 >> comma >> weight) {
                    Node node2{ lon2, lat2 };
                    adjList[node1].emplace_back(node2, weight);
                    adjList[node2].emplace_back(node1, weight);
                    if (!(lineStream >> comma) || comma != ';') {
                        break;
                    }
                }
            }
        }
    }

    // Метод для очистки графа
    void clear() {
        adjList.clear();
    }

    // Метод для выполнения поиска в ширину (BFS)
    pair<pair<double, int>, vector<Node>> bfs(const Node& start, const Node& goal) {
        // Очередь для хранения вершин и их текущих расстояний
        queue<pair<Node, double>> q;

        // Хеш-таблицы для отслеживания посещенных вершин, расстояний и родительских вершин
        unordered_map<Node, bool> visited;
        unordered_map<Node, double> distance;
        unordered_map<Node, Node> parent;

        // Начальная установка: добавляем начальную вершину в очередь с расстоянием 0
        q.push({ start, 0.0 });
        visited[start] = true;
        distance[start] = 0.0;
        parent[start] = start;

        // Основной цикл BFS
        while (!q.empty()) {
            // Извлекаем текущую вершину и её расстояние из очереди
            Node current = q.front().first;
            double dist = q.front().second;
            q.pop();

            // Если текущая вершина совпадает с целевой вершиной, восстанавливаем путь
            if (current == goal) {
                vector<Node> path;
                int edgeCount = 0;

                // Восстановление пути от goal до start
                for (Node at = goal; at != start; at = parent[at]) {
                    path.push_back(at);
                    edgeCount++;
                }
                path.push_back(start);

                // Обращение пути, чтобы он начинался с start и заканчивался goal
                reverse(path.begin(), path.end());

                // Возвращаем результат: расстояние, количество ребер и путь
                return { { dist, edgeCount }, path };
            }

            // Исследование соседних вершин текущей вершины
            for (const auto& neighbor_pair : adjList[current]) {
                const Node& neighbor = neighbor_pair.first;
                double weight = neighbor_pair.second;

                // Если соседняя вершина не была посещена
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    distance[neighbor] = dist + weight;
                    parent[neighbor] = current;
                    q.push({ neighbor, dist + weight });
                }
            }
        }

        // Если целевая вершина не была найдена, возвращаем бесконечное расстояние и пустой путь
        return { { numeric_limits<double>::infinity(), 0 }, {} };
    }

    // Метод для выполнения поиска в глубину (DFS)
    pair<pair<double, int>, vector<Node>> dfs(const Node& start, const Node& goal) {
        // Стек для хранения вершин и их текущих расстояний
        stack<pair<Node, double>> s;

        // Хеш-таблицы для отслеживания посещенных вершин, расстояний и родительских вершин
        unordered_map<Node, bool> visited;
        unordered_map<Node, double> distance;
        unordered_map<Node, Node> parent;

        // Начальная установка: добавляем начальную вершину в стек с расстоянием 0
        s.push({ start, 0.0 });
        visited[start] = true;
        distance[start] = 0.0;
        parent[start] = start;

        // Основной цикл DFS
        while (!s.empty()) {
            // Извлекаем текущую вершину и её расстояние из стека
            Node current = s.top().first;
            double dist = s.top().second;
            s.pop();

            // Если текущая вершина совпадает с целевой вершиной, восстанавливаем путь
            if (current == goal) {
                vector<Node> path;
                int edgeCount = 0;

                // Восстановление пути от goal до start
                for (Node at = goal; at != start; at = parent[at]) {
                    path.push_back(at);
                    edgeCount++;
                }
                path.push_back(start);

                // Обращение пути, чтобы он начинался с start и заканчивался goal
                reverse(path.begin(), path.end());

                // Возвращаем результат: расстояние, количество ребер и путь
                return { {dist, edgeCount}, path };
            }

            // Исследование соседних вершин текущей вершины
            for (const auto& neighbor_pair : adjList[current]) {
                const Node& neighbor = neighbor_pair.first;
                double weight = neighbor_pair.second;

                // Если соседняя вершина не была посещена
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    distance[neighbor] = dist + weight;
                    parent[neighbor] = current;
                    s.push({ neighbor, dist + weight });
                }
            }
        }

        // Если целевая вершина не была найдена, возвращаем бесконечное расстояние и пустой путь
        return { {numeric_limits<double>::infinity(), 0}, {} };
    }

    // Метод для выполнения алгоритма Дейкстры
    pair<double, vector<Node>> dijkstra(const Node& start, const Node& goal) {
        // Определение типа для пары (расстояние, узел)
        using pii = pair<double, Node>;

        // Приоритетная очередь для хранения узлов и их текущих расстояний
        priority_queue<pii, vector<pii>, greater<pii>> pq;

        // Хеш-таблицы для хранения расстояний, посещенных узлов и родительских узлов
        unordered_map<Node, double> distances;
        unordered_map<Node, bool> visited;
        unordered_map<Node, Node> parent;

        // Инициализация расстояний до всех узлов как бесконечность
        for (const auto& pair : adjList) {
            distances[pair.first] = numeric_limits<double>::infinity();
        }

        // Установка начального расстояния до стартового узла
        distances[start] = 0.0;
        pq.push({ 0.0, start });
        parent[start] = start;

        // Основной цикл алгоритма Дейкстры
        while (!pq.empty()) {
            // Извлечение узла с минимальным расстоянием
            double currentDistance = pq.top().first;
            Node current = pq.top().second;
            pq.pop();

            // Если текущий узел совпадает с целевым узлом, восстанавливаем путь
            if (current == goal) {
                vector<Node> path;
                for (Node at = goal; at != start; at = parent[at]) {
                    path.push_back(at);
                }
                path.push_back(start);
                reverse(path.begin(), path.end());
                return { currentDistance, path };
            }

            // Если узел уже посещен, пропускаем его
            if (visited[current]) continue;
            visited[current] = true;

            // Исследование соседних узлов
            for (const auto& neighbor_pair : adjList[current]) {
                const Node& neighbor = neighbor_pair.first;
                double weight = neighbor_pair.second;

                // Вычисление нового расстояния до соседнего узла
                double newDist = currentDistance + weight;

                // Если новое расстояние меньше текущего расстояния до соседнего узла
                if (newDist < distances[neighbor]) {
                    distances[neighbor] = newDist;
                    parent[neighbor] = current;
                    pq.push({ newDist, neighbor });
                }
            }
        }

        // Если целевой узел не был найден, возвращаем бесконечное расстояние и пустой путь
        return { numeric_limits<double>::infinity(), {} };
    }

    // Метод для выполнения алгоритма A*
    pair<double, vector<Node>> aStar(const Node& start, const Node& goal) {
        // Эвристическая функция для оценки расстояния между двумя узлами
        auto heuristic = [](const Node& a, const Node& b) {
            return sqrt(pow(a.lon - b.lon, 2) + pow(a.lat - b.lat, 2));
            };

        // Определение типа для пары (расстояние, узел)
        using pii = pair<double, Node>;

        // Приоритетная очередь для хранения узлов и их текущих fScore
        priority_queue<pii, vector<pii>, greater<pii>> pq;

        // Хеш-таблицы для хранения gScore, fScore и родительских узлов
        unordered_map<Node, double> gScore;
        unordered_map<Node, double> fScore;
        unordered_map<Node, Node> parent;

        // Инициализация gScore и fScore для всех узлов как бесконечность
        for (const auto& pair : adjList) {
            gScore[pair.first] = numeric_limits<double>::infinity();
            fScore[pair.first] = numeric_limits<double>::infinity();
        }

        // Установка начального gScore и fScore для стартового узла
        gScore[start] = 0.0;
        fScore[start] = heuristic(start, goal);
        parent[start] = start;

        // Добавление стартового узла в приоритетную очередь
        pq.push({ fScore[start], start });

        // Основной цикл алгоритма A*
        while (!pq.empty()) {
            // Извлечение узла с минимальным fScore
            double currentFScore = pq.top().first;
            Node current = pq.top().second;
            pq.pop();

            // Если текущий узел совпадает с целевым узлом, восстанавливаем путь
            if (current == goal) {
                vector<Node> path;
                for (Node at = goal; at != start; at = parent[at]) {
                    path.push_back(at);
                }
                path.push_back(start);
                reverse(path.begin(), path.end());
                return { gScore[current], path };
            }

            // Исследование соседних узлов
            for (const auto& neighbor_pair : adjList[current]) {
                const Node& neighbor = neighbor_pair.first;
                double weight = neighbor_pair.second;

                // Вычисление временного gScore для соседнего узла
                double tentativeGScore = gScore[current] + weight;

                // Если временный gScore меньше текущего gScore для соседнего узла
                if (tentativeGScore < gScore[neighbor]) {
                    // Обновление gScore и fScore для соседнего узла
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = tentativeGScore + heuristic(neighbor, goal);
                    parent[neighbor] = current;
                    pq.push({ fScore[neighbor], neighbor });
                }
            }
        }

        // Если целевой узел не был найден, возвращаем бесконечное расстояние и пустой путь
        return { numeric_limits<double>::infinity(), {} };
    }

private:
    // Список смежности графа
    unordered_map<Node, vector<pair<Node, double>>> adjList;
};

// Функция для запуска тестов
void runTests() {
    Graph graph;

    // Тест 1: Простой граф с прямым путем
    string data1 = R"(
        1.0,1.0:2.0,2.0,1.0;
        2.0,2.0:3.0,3.0,1.0;
        3.0,3.0:4.0,4.0,1.0;
    )";
    graph.parseData(data1);

    Node start1{ 1.0, 1.0 };
    Node goal1{ 4.0, 4.0 };
    assert(graph.bfs(start1, goal1).first.first == 3.0);
    assert(graph.dfs(start1, goal1).first.first == 3.0);
    assert(graph.dijkstra(start1, goal1).first == 3.0);
    assert(graph.aStar(start1, goal1).first == 3.0);

    graph.clear();

    // Тест 2: Граф без пути
    string data2 = R"(
        1.0,1.0:2.0,2.0,1.0;
        3.0,3.0:4.0,4.0,1.0;
    )";
    graph.parseData(data2);

    Node start2{ 1.0, 1.0 };
    Node goal2{ 4.0, 4.0 };

    assert(graph.bfs(start2, goal2).first.first == numeric_limits<double>::infinity());
    assert(graph.dfs(start2, goal2).first.first == numeric_limits<double>::infinity());
    assert(graph.dijkstra(start2, goal2).first == numeric_limits<double>::infinity());
    assert(graph.aStar(start2, goal2).first == numeric_limits<double>::infinity());

    graph.clear();

    // Тест 3: Граф с циклом
    string data3 = R"(
        1.0,1.0:2.0,2.0,1.0;
        2.0,2.0:3.0,3.0,1.0;
        3.0,3.0:1.0,1.0,1.0;
    )";
    graph.parseData(data3);

    Node start3{ 1.0, 1.0 };
    Node goal3{ 3.0, 3.0 };

    assert(graph.bfs(start3, goal3).first.first == 1.0);
    assert(graph.dfs(start3, goal3).first.first == 1.0);
    assert(graph.dijkstra(start3, goal3).first == 1.0);
    assert(graph.aStar(start3, goal3).first == 1.0);

    cout << "All tests passed!" << endl;
}

// Функция для тестирования данных из файла
void test_file() {
    Graph graph;

    // Чтение данных из файла
    ifstream inputFile("graph.txt");
    if (!inputFile.is_open()) {
        cerr << "Не удалось открыть файл!" << endl;
        return;
    }

    stringstream buffer;
    buffer << inputFile.rdbuf();
    string data = buffer.str();
    graph.parseData(data);

    Node start{ 1.0, 1.0 };
    Node goal{ 5.0, 5.0 };

    assert(graph.bfs(start, goal).first.first == 3.0);
    assert(graph.dfs(start, goal).first.first == 3.0);
    assert(graph.dijkstra(start, goal).first == 3.0);
    assert(graph.aStar(start, goal).first == 3.0);

    cout << "Test_file passed!" << endl;
}

// Функция для тестирования данных из другого файла
void test_file2() {
    Graph graph;

    // Чтение данных из файла
    ifstream inputFile("graph2.txt");
    if (!inputFile.is_open()) {
        cerr << "Не удалось открыть файл!" << endl;
        return;
    }

    stringstream buffer;
    buffer << inputFile.rdbuf();
    string data = buffer.str();
    graph.parseData(data);

    Node start{ 1.0, 1.0 }; // Стартовая вершина
    Node goal{ 10.0, 10.0 };  // Целевая вершина

    // Запуск алгоритмов и замер времени
    auto startBFS = chrono::high_resolution_clock::now();
    auto bfsResult = graph.bfs(start, goal);
    auto endBFS = chrono::high_resolution_clock::now();
    cout << "BFS Distance: " << bfsResult.first.first << " Edges: " << bfsResult.first.second << " Time: "
        << chrono::duration_cast<chrono::microseconds>(endBFS - startBFS).count() << "ms" << endl;
    cout << "BFS Path: ";
    for (const auto& node : bfsResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") ";
    }
    cout << endl;

    auto startDFS = chrono::high_resolution_clock::now();
    auto dfsResult = graph.dfs(start, goal);
    auto endDFS = chrono::high_resolution_clock::now();
    cout << "DFS Distance: " << dfsResult.first.first << " Edges: " << dfsResult.first.second << " Time: "
        << chrono::duration_cast<chrono::microseconds>(endDFS - startDFS).count() << "ms" << endl;
    cout << "DFS Path: ";
    for (const auto& node : dfsResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") ";
    }
    cout << endl;

    auto startDijkstra = chrono::high_resolution_clock::now();
    auto dijkstraResult = graph.dijkstra(start, goal);
    auto endDijkstra = chrono::high_resolution_clock::now();
    cout << "Dijkstra Distance: " << dijkstraResult.first << " Time: "
        << chrono::duration_cast<chrono::microseconds>(endDijkstra - startDijkstra).count() << "ms" << endl;
    cout << "Dijkstra Path: ";
    for (const auto& node : dijkstraResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") ";
    }
    cout << endl;

    auto startAStar = chrono::high_resolution_clock::now();
    auto aStarResult = graph.aStar(start, goal);
    auto endAStar = chrono::high_resolution_clock::now();
    cout << "A* Distance: " << aStarResult.first << " Time: "
        << chrono::duration_cast<chrono::microseconds>(endAStar - startAStar).count() << "ms" << endl;
    cout << "A* Path: ";
    for (const auto& node : aStarResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") ";
    }
    cout << endl;
}

int main() {
    runTests();
    test_file();
    test_file2();

    Graph graph;

    // Чтение данных из файла
    ifstream inputFile("spb_graph.txt");
    if (!inputFile.is_open()) {
        cerr << "Не удалось открыть файл!" << endl;
        return 1;
    }
    cout << "File open!" << endl;

    stringstream buffer;
    buffer << inputFile.rdbuf();
    string data = buffer.str();
    graph.parseData(data);

    Node start{ 30.63432, 60.024664 }; // Стартовая вершина
    Node goal{ 30.3122481, 59.9563633 };  // Целевая вершина

    // Запуск алгоритмов и замер времени
    auto startBFS = chrono::high_resolution_clock::now();
    auto bfsResult = graph.bfs(start, goal);
    auto endBFS = chrono::high_resolution_clock::now();
    cout << "BFS Distance: " << bfsResult.first.first << " Edges: " << bfsResult.first.second << " Time: "
        << chrono::duration_cast<chrono::microseconds>(endBFS - startBFS).count() << "ms" << endl;
    cout << endl;

    auto startDFS = chrono::high_resolution_clock::now();
    auto dfsResult = graph.dfs(start, goal);
    auto endDFS = chrono::high_resolution_clock::now();
    cout << "DFS Distance: " << dfsResult.first.first << " Edges: " << dfsResult.first.second << " Time: "
        << chrono::duration_cast<chrono::microseconds>(endDFS - startDFS).count() << "ms" << endl;
    cout << endl;

    auto startDijkstra = chrono::high_resolution_clock::now();
    auto dijkstraResult = graph.dijkstra(start, goal);
    auto endDijkstra = chrono::high_resolution_clock::now();
    cout << "Dijkstra Distance: " << dijkstraResult.first << " Time: "
        << chrono::duration_cast<chrono::microseconds>(endDijkstra - startDijkstra).count() << "ms" << endl;
    cout << endl;

    auto startAStar = chrono::high_resolution_clock::now();
    auto aStarResult = graph.aStar(start, goal);
    auto endAStar = chrono::high_resolution_clock::now();
    cout << "A* Distance: " << aStarResult.first << " Time: "
        << chrono::duration_cast<chrono::microseconds>(endAStar - startAStar).count() << "ms" << endl;
    cout << endl;

    /*cout << "BFS Path: ";
    for (const auto& node : bfsResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") ";
    }
    cout << endl;

    cout << "DFS Path: ";
    for (const auto& node : dfsResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") ";
    }
    cout << endl;

    cout << "Dijkstra Path: ";
    for (const auto& node : dijkstraResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") ";
    }
    cout << endl;

    cout << "A* Path: ";
    for (const auto& node : aStarResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") ";
    }
    cout << endl;*/

    return 0;
}
