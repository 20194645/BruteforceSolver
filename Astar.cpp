#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <tuple>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <set>
#include <map>
#include <boost/functional/hash.hpp>

using namespace std;

vector<vector<int>> starts;
vector<int> goals;
map<int, int> spacegoals;
int numberAGVs, N, objV;
vector<vector<pair<int, int>>> table;
vector<vector<int>> marked;
struct State
{
    int car, cost, f;
    vector<vector<int>> path;
    unordered_set<pair<int, int>, boost::hash<pair<int, int>>> used_edges; // Dùng unordered_set
    bool operator<(const State &other) const
    {
        return f > other.f; // Min-heap
    }
};

vector<vector<pair<int, int>>> spacegraph;

void readSpacemap()
{
    spacegraph.resize(101);
    ifstream file("/mnt/d/DATN/pathPlanningSimulation/spacemap.txt");
    string line;
    if (!file.is_open())
    {
        cerr << "Không thể mở file!" << endl;
        return;
    }
    while (getline(file, line))
    {
        if (line[0] == 'a' && line[1] == ' ')
        {
            istringstream iss(line);
            char label;
            int id1, id2, upper, lower, weight;
            if (iss >> label >> id1 >> id2 >> upper >> lower >> weight)
            {
                N = max(N, max(id1, id2));
                spacegraph[id1].push_back(make_pair(id2, weight));
            }
            else
            {
                cerr << "Dòng không hợp lệ: " << line << endl;
            }
        }
    }

    file.close();
}

vector<int> dijkstra(int V, const vector<vector<pair<int, int>>> &graph, int start)
{
    vector<int> dist(V, numeric_limits<int>::max());
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty())
    {
        auto [currentDist, currentNode] = pq.top();
        pq.pop();

        if (currentDist > dist[currentNode])
            continue;

        for (const auto &edge : graph[currentNode])
        {
            int neighbor = edge.first;
            int weight = edge.second;

            int newDist = dist[currentNode] + weight;

            if (newDist < dist[neighbor])
            {
                dist[neighbor] = newDist;
                pq.push({newDist, neighbor});
            }
        }
    }
    return dist;
}

void generateEdgeTable(int goal)
{
    table.resize(N + 1);
    for (int start = 1; start <= N; ++start)
    {
        vector<int> distances = dijkstra(N + 1, spacegraph, start);
        if (distances[goal] != numeric_limits<int>::max())
        {
            table[start].push_back(make_pair(goal, distances[goal]));
        }
    }
}

vector<vector<pair<int, int>>> graph;

void readTSGmap()
{
    ifstream file("/mnt/d/DATN/pathPlanningSimulation/TSG.txt");
    string line;
    int agv_id = 0;
    while (getline(file, line))
    {
        if (line.empty())
            continue;
        if (line[0] == 'p')
        {
            string tmp, tmp1, number_of_node, number_of_edge;
            stringstream ss(line);
            ss >> tmp >> tmp1 >> number_of_node >> number_of_edge;
            graph.resize(stoi(number_of_node) + 1);
        }
        else if (line[0] == 'c' && line[2] == 'n')
        {
            istringstream ss(line);
            string tmp1, tmp2;
            int end_node, flow, earliness, tardiness, TW_node;
            ss >> tmp1 >> tmp2 >> end_node >> flow >> earliness >> tardiness >> TW_node;
            spacegoals[TW_node] = end_node;
        }
        else if (line[0] == 'n')
        {
            string tmp, node, flow;
            stringstream ss(line);
            ss >> tmp >> node >> flow;
            if (stoi(flow) == -1)
            {
                goals.push_back(stoi(node));
            }
            else if (stoi(flow) == 1)
            {
                starts[agv_id].push_back(stoi(node));
                agv_id++;
            }
        }
        else if (line[0] == 'a' and line[1] == ' ')
        {
            string tmp, start_node, end_node, lower, upper, weight;
            stringstream ss(line);
            ss >> tmp >> start_node >> end_node >> lower >> upper >> weight;
            graph[stoi(start_node)].push_back(make_pair(stoi(end_node), stoi(weight)));
        }
    }
    numberAGVs = agv_id;
    file.close();
}

bool check(vector<vector<int>> path)
{
    for (int i = 0; i < numberAGVs; i++)
    {
        if (path[i].back() != goals[i])
        {
            return false;
        }
    }
    return true;
}

int heuristic(vector<vector<int>> path)
{
    int H = 0;
    for (int i = 0; i < numberAGVs; i++)
    {
        if (path[i].back() != goals[i])
        {
            // Kiểm tra nếu path[i].back() % N == 0 thì gán space_current = N
            int space_current = path[i].back() % N == 0 ? N : path[i].back() % N;

            int min_distance;
            int time = (path[i].back() % N == 0) ? (path[i].back() / N - 1) : (path[i].back() / N);
            for (auto pair : table[space_current])
            {
                if (pair.first == spacegoals[goals[i]])
                {
                    min_distance = pair.second;
                }
            }
            int estimate_Time = min_distance + time;
            int end_node = estimate_Time * N + spacegoals[goals[i]];
            
            int TW_penalty = 9999999;
            if (end_node < graph.size())
            {
                for (auto pair : graph[end_node])
                {

                    if (pair.first == goals[i])
                    {
                        TW_penalty = pair.second;
                    }
                }
            }
            H = H + min_distance + TW_penalty;
        }
    }
    return H;
}

void a_star()
{
    priority_queue<State> pq;
    unordered_set<pair<int, int>, boost::hash<pair<int, int>>> used_edges;
    State root = {0, 0, 0, starts, used_edges};
    pq.push(root);

    while (!pq.empty())
    {
        State current = pq.top();
        pq.pop();

        if (check(current.path))
        {
            objV = current.cost;
            marked.resize(numberAGVs);
            for (int i = 0; i < numberAGVs; i++)
            {
                marked[i] = current.path[i];
            }
            return;
        }

        if (current.path[current.car].back() == goals[current.car])
        {
            current.car = (current.car + 1) % numberAGVs;
            pq.push(current);
            continue;
        }

        int current_node = current.path[current.car].back();
        for (auto e : graph[current_node])
        {
            if (current.used_edges.find({current_node, e.first}) == current.used_edges.end())
            {
                State next = current;
                next.path[next.car].push_back(e.first);
                next.cost += e.second;
                next.car = (next.car + 1) % numberAGVs;
                next.used_edges.insert({current_node, e.first});
                next.f = next.cost + heuristic(next.path);
                pq.push(next);
            }
        }
    }
     // Giải phóng bộ nhớ
    pq = priority_queue<State>();  // Xóa priority queue
    used_edges.clear();  // Giải phóng unordered_set
}

int main()
{
    starts.resize(100);
    readSpacemap();
    readTSGmap();
    for (auto pair : spacegoals)
    {
        generateEdgeTable(pair.second);
    }
    a_star();
    cout <<"ObjV: " <<objV << endl;
    for (int i = 0; i < numberAGVs; i++) {
        cout<<"Routing AGV "<<i<<": ";
        for (auto node : marked[i]) {
            cout << node << " ";
        }
        cout << endl;
    }

    // Giải phóng bộ nhớ cho starts, goals, marked sau khi in ra
    starts.clear();
    goals.clear();
    marked.clear();
    return 0;
}
