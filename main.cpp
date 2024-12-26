#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>
using namespace std;
vector<vector<int>> path;
vector<vector<int>> marked;
vector<vector<pair<int, int>>> graph;
vector<int> destination;
int numberAGVs;
int N = -1;
int tmp = 999999999;
unordered_map<int, unordered_map<int, bool>> visited;

void readSpacemap()
{
    ifstream file("/mnt/d/DATN/pathPlanningSimulation/spacemap.txt");
    string line;
    if (!file.is_open())
    {
        std::cerr << "Không thể mở file!" << std::endl;
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
                N = max(N, id1);
                N = max(N, id2);
            }
            else
            {
                std::cerr << "Dòng không hợp lệ: " << line << std::endl;
            }
        }
    }
}
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
        if (line[0] == 'n')
        {
            string tmp, node, flow;
            stringstream ss(line);
            ss >> tmp >> node >> flow;
            if (stoi(flow) == -1)
            {
                destination.push_back(stoi(node));
            }
            else if (stoi(flow) == 1)
            {
                path[agv_id].push_back(stoi(node));
                agv_id++;
            }
        }
        else if (line[0] == 'a')
        {
            string tmp, start_node, end_node, lower, upper, weight;
            stringstream ss(line);
            ss >> tmp >> start_node >> end_node >> lower >> upper >> weight;
            graph[stoi(start_node)].push_back(make_pair(stoi(end_node), stoi(weight)));
        }
    }
    numberAGVs = agv_id;
}

bool check(int current_node, int next_node)
{
    return !visited[current_node][next_node];
}

void solve(int id, int cost, set<int> complete_agv, vector<int> des)
{
    if (cost >= tmp)
    {
        return;
    }

    if (complete_agv.size() == numberAGVs)
    {
        if (cost < tmp)
        {
            tmp = cost;
            for (int i = 0; i < numberAGVs; i++)
            {
                marked[i].clear();
                for (int j = 0; j < path[i].size(); j++)
                {
                    marked[i].push_back(path[i][j]);
                }
            }
        }
    }
    else
    {
        if (complete_agv.find(id) == complete_agv.end())
        {
            int current_node = path[id].back();
            sort(graph[current_node].begin(), graph[current_node].end(), [](const pair<int, int> &a, const pair<int, int> &b)
                 {
                     return a.second < b.second; // So sánh trọng số
                 });
            for (auto e : graph[current_node])
            {
                if (check(current_node, e.first))
                {
                    visited[current_node][e.first] = true;
                    path[id].push_back(e.first);
                    if (find(des.begin(), des.end(), e.first) != des.end())
                    {
                        complete_agv.insert(id);
                        auto it = find(des.begin(), des.end(), e.first);
                        des.erase(it);
                    }

                    if (cost <= tmp)
                    {
                        solve((id + 1) % numberAGVs, cost + e.second, complete_agv, des);
                    }
                    path[id].pop_back();
                    visited[current_node][e.first] = false;
                }
            }
        }
        else
        {
            solve((id + 1) % numberAGVs, cost, complete_agv, des);
        }
    }
}

int main()
{
    path.resize(100);
    marked.resize(100);
    readSpacemap();
    readTSGmap();
    set<int> complete_agv;

    solve(0, 0, complete_agv, destination);
    cout <<"ObjV: " <<tmp << endl;
    for (int i = 0; i < numberAGVs; i++)
    {
        cout<<"Routing of AGV "<<i<<" :";
        for (auto node : marked[i])
        {
            cout << node << " ";
        }
        cout << endl;
    }
}