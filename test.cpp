#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>

using namespace std;

const int INF = numeric_limits<int>::max();

void reduceRows(vector<vector<int>> &cost, int N) {
    for (int i = 0; i < N; i++) {
        int rowMin = *min_element(cost[i].begin(), cost[i].end());
        for (int j = 0; j < N; j++) {
            cost[i][j] -= rowMin;
        }
    }
}

void reduceColumns(vector<vector<int>> &cost, int N) {
    for (int j = 0; j < N; j++) {
        int colMin = INF;
        for (int i = 0; i < N; i++) {
            colMin = min(colMin, cost[i][j]);
        }
        for (int i = 0; i < N; i++) {
            cost[i][j] -= colMin;
        }
    }
}

bool findMatch(int worker, const vector<vector<int>> &cost, vector<int> &matchJob, vector<bool> &visited, int N) {
    for (int job = 0; job < N; job++) {
        if (cost[worker][job] == 0 && !visited[job]) {
            visited[job] = true;
            if (matchJob[job] == -1 || findMatch(matchJob[job], cost, matchJob, visited, N)) {
                matchJob[job] = worker;
                return true;
            }
        }
    }
    return false;
}

int hungarianAlgorithm(const vector<vector<int>> &originalCost) {
    int N = originalCost.size();
    vector<vector<int>> cost = originalCost; // Tạo bản sao của ma trận gốc

    // Step 1: Reduce rows
    reduceRows(cost, N);

    // Step 2: Reduce columns
    reduceColumns(cost, N);

    // Step 3: Find optimal assignment using DFS-based bipartite matching
    vector<int> matchJob(N, -1); // matchJob[j] = worker assigned to job j

    for (int worker = 0; worker < N; worker++) {
        vector<bool> visited(N, false);
        findMatch(worker, cost, matchJob, visited, N);
    }

    // Step 4: Calculate minimum cost using the original cost matrix
    int totalCost = 0;
    for (int job = 0; job < N; job++) {
        if (matchJob[job] != -1) {
            totalCost += originalCost[matchJob[job]][job];
        }
    }
    return totalCost;
}

int main() {
    // Example input: cost matrix
    vector<vector<int>> cost = {
        {4, 2, 8},
        {2, 3, 7},
        {6, 4, 3}
    };

    // Run Hungarian Algorithm
    int result = hungarianAlgorithm(cost);

    cout << "The minimum cost is: " << result << endl;

    return 0;
}
