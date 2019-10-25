class Solution {
public:
    vector<vector<int>> criticalConnections(int n, vector<vector<int>>& connections) {
        vector<vector<bool>> table(n, vector<bool> (n, false));
        vector<vector<int>> res; 
        for(auto& v : connections){
            table[v[0]][v[1]] = true; 
            table[v[1]][v[0]] = true; 
        }
        for(auto& v : connections){
            vector<bool> visit(n, false); 
            table[v[0]][v[1]] = false;
            table[v[1]][v[0]] = false; 
            visit[v[1]] = true;
            helper(table, v[1], visit); 
            for(int i = 0; i < visit.size(); i++){
                if(!visit[i]){
                    res.push_back(v); 
                    break;
                }
            }
            table[v[0]][v[1]] = true;
            table[v[1]][v[0]] = true; 
        }
        return res; 
    }
    void helper(vector<vector<bool>>& table, int i, vector<bool>& visit){
        for(int j = 0; j < table.size(); j++){
            if(table[i][j] && !visit[j]){
                table[j][i] = false; 
                visit[j] = true; 
                helper(table, j, visit); 
                table[j][i] = true; 
            }
        }
    }
};