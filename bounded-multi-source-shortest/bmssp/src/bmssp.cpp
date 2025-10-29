#include "./../include/bmssp.h"
#include "./../include/data_structure_d.h"
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <vector>

using namespace std;

pair<unordered_set<Node>, unordered_set<Node>> find_pivots(
    const Graph& graph,
    unordered_map<Node, Weight>& dist,
    const unordered_set<Node>& S,
    double B, int n, int k_steps, int p_limit,
    Instrument* instr) {
    
    Instrument local_instr;
    if (!instr) instr = &local_instr;
    
    vector<Node> S_filtered;
    for (Node v : S) {
        if (dist[v] < B) {
            S_filtered.push_back(v);
        }
    }
    
    unordered_set<Node> P;
    if (S_filtered.empty()) {
        int count = 0;
        for (Node v : S) {
            if (count++ >= max(1, min((int)S.size(), p_limit))) break;
            P.insert(v);
        }
    } else {
        sort(S_filtered.begin(), S_filtered.end(),
             [&dist](Node a, Node b) { return dist[a] < dist[b]; });
        int limit = max(1, min((int)S_filtered.size(), p_limit));
        for (int i = 0; i < limit; ++i) {
            P.insert(S_filtered[i]);
        }
    }
    
    unordered_set<Node> source_frontier = P.empty() ? S : P;
    unordered_set<Node> discovered = source_frontier;
    unordered_set<Node> frontier = source_frontier;
    
    for (int step = 0; step < max(1, k_steps); ++step) {
        if (frontier.empty()) break;
        
        unordered_set<Node> next_front;
        for (Node u : frontier) {
            Weight du = dist[u];
            if (du >= B) continue;
            
            if (graph.find(u) != graph.end()) {
                for (const auto& [v, w] : graph.at(u)) {
                    instr->relaxations++;
                    Weight nd = du + w;
                    if (nd < B && discovered.find(v) == discovered.end()) {
                        discovered.insert(v);
                        next_front.insert(v);
                    }
                }
            }
        }
        frontier = next_front;
    }
    
    unordered_set<Node> W = discovered;
    if (P.empty() && !S.empty()) {
        P.insert(*S.begin());
    }
    
    return {P, W};
}

pair<double, unordered_set<Node>> basecase(
    const Graph& graph,
    unordered_map<Node, Weight>& dist,
    double B,
    const unordered_set<Node>& S,
    int k,
    Instrument* instr) {
    
    Instrument local_instr;
    if (!instr) instr = &local_instr;
    
    if (S.empty()) {
        return {B, unordered_set<Node>()};
    }
    
    Node x = *min_element(S.begin(), S.end(),
                          [&dist](Node a, Node b) { return dist[a] < dist[b]; });
    
    using PQPair = pair<Weight, Node>;
    priority_queue<PQPair, vector<PQPair>, greater<PQPair>> heap;
    
    Weight start_d = dist[x];
    heap.push({start_d, x});
    instr->heap_ops++;
    
    unordered_set<Node> Uo;
    
    while (!heap.empty() && (int)Uo.size() < (k + 1)) {
        auto [d_u, u] = heap.top();
        heap.pop();
        instr->heap_ops++;
        
        if (d_u > dist[u]) continue;
        
        Uo.insert(u);
        
        if (graph.find(u) != graph.end()) {
            for (const auto& [v, w] : graph.at(u)) {
                instr->relaxations++;
                Weight newd = dist[u] + w;
                if (newd < dist[v] && newd < B) {
                    dist[v] = newd;
                    heap.push({newd, v});
                    instr->heap_ops++;
                }
            }
        }
    }
    
    if ((int)Uo.size() <= k) {
        return {B, Uo};
    } else {
        vector<Weight> finite_dists;
        for (Node v : Uo) {
            if (isfinite(dist[v])) {
                finite_dists.push_back(dist[v]);
            }
        }
        
        if (finite_dists.empty()) {
            return {B, unordered_set<Node>()};
        }
        
        Weight maxd = *max_element(finite_dists.begin(), finite_dists.end());
        unordered_set<Node> U_filtered;
        for (Node v : Uo) {
            if (dist[v] < maxd) {
                U_filtered.insert(v);
            }
        }
        return {maxd, U_filtered};
    }
}

pair<double, unordered_set<Node>> bmssp(
    const Graph& graph,
    unordered_map<Node, Weight>& dist,
    const vector<Edge>& edges,
    int l, double B,
    const unordered_set<Node>& S,
    int n,
    Instrument* instr) {
    
    Instrument local_instr;
    if (!instr) instr = &local_instr;
    
    int t_param, k_param;
    if (n <= 2) {
        t_param = 1;
        k_param = 2;
    } else {
        t_param = max(1, (int)round(pow(log(max(3, n)), 2.0 / 3.0)));
        k_param = max(2, (int)round(pow(log(max(3, n)), 1.0 / 3.0)));
    }
    
    if (l <= 0) {
        if (S.empty()) {
            return {B, unordered_set<Node>()};
        }
        return basecase(graph, dist, B, S, k_param, instr);
    }
    
    int p_limit = max(1, 1 << min(10, t_param));
    int k_steps = max(1, k_param);
    
    auto [P, W] = find_pivots(graph, dist, S, B, n, k_steps, p_limit, instr);
    
    int M = 1 << max(0, (l - 1) * t_param);
    DataStructureD D(M, B, max(1, min((int)P.size(), 64)));
    
    for (Node x : P) {
        D.insert(x, dist[x]);
    }
    
    double B_prime_initial = B;
    if (!P.empty()) {
        B_prime_initial = numeric_limits<double>::infinity();
        for (Node x : P) {
            B_prime_initial = min(B_prime_initial, dist[x]);
        }
    }
    
    unordered_set<Node> U;
    vector<double> B_prime_sub_values;
    
    int loop_guard = 0;
    int limit = k_param * (1 << (l * max(1, t_param)));
    
    while ((int)U.size() < limit && !D.empty()) {
        if (++loop_guard > 20000) break;
        
        double Bi;
        unordered_set<Node> Si;
        try {
            auto result = D.pull();
            Bi = result.first;
            Si = result.second;
        } catch (...) {
            break;
        }
        
        auto [B_prime_sub, Ui] = bmssp(graph, dist, edges, l - 1, Bi, Si, n, instr);
        B_prime_sub_values.push_back(B_prime_sub);
        
        U.insert(Ui.begin(), Ui.end());
        
        vector<pair<Node, Weight>> K_for_batch;
        for (Node u : Ui) {
            Weight du = dist[u];
            if (!isfinite(du)) continue;
            
            if (graph.find(u) != graph.end()) {
                for (const auto& [v, w_uv] : graph.at(u)) {
                    instr->relaxations++;
                    Weight newd = du + w_uv;
                    if (newd <= dist[v]) {
                        dist[v] = newd;
                        if (Bi <= newd && newd < B) {
                            D.insert(v, newd);
                        } else if (B_prime_sub <= newd && newd < Bi) {
                            K_for_batch.push_back({v, newd});
                        }
                    }
                }
            }
        }
        
        for (Node x : Si) {
            Weight dx = dist[x];
            if (B_prime_sub <= dx && dx < Bi) {
                K_for_batch.push_back({x, dx});
            }
        }
        
        if (!K_for_batch.empty()) {
            D.batch_prepend(K_for_batch);
        }
    }
    
    double B_prime_final = B_prime_initial;
    if (!B_prime_sub_values.empty()) {
        B_prime_final = min(B_prime_initial,
                            *min_element(B_prime_sub_values.begin(),
                                         B_prime_sub_values.end()));
    }
    
    unordered_set<Node> U_final = U;
    for (Node x : W) {
        if (dist[x] < B_prime_final) {
            U_final.insert(x);
        }
    }
    
    return {B_prime_final, U_final};
}
