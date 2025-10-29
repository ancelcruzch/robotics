#include "./../include/types.h"
#include "./../include/graph_generator.h"
#include "./../include/dijkstra.h"
#include "./../include/bmssp.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstdlib>   // para atoi, atof

using namespace std;

struct BenchResult { 
    double time_dij; 
    double time_bm; 
};

static BenchResult run_benchmark(const Graph& G, const vector<Edge>& E, Node source) {
    // Dijkstra
    Instrument instr_dij;
    auto t0 = chrono::high_resolution_clock::now();
    auto dist_dij = dijkstra(G, source, &instr_dij);
    auto t1 = chrono::high_resolution_clock::now();
    double time_dij = chrono::duration<double>(t1 - t0).count();

    // BMSSP
    unordered_map<Node, Weight> dist_bm;
    dist_bm.reserve(G.size());
    for (const auto& [u, _] : G) dist_bm[u] = numeric_limits<Weight>::infinity();
    dist_bm[source] = 0.0;

    int n_nodes = static_cast<int>(G.size());
    int l;
    if (n_nodes <= 2) l = 1;
    else {
        int t_guess = max(1, static_cast<int>(round(pow(log(max(3, n_nodes)), 2.0 / 3.0))));
        l = max(1, static_cast<int>(round(log(max(3, n_nodes)) / t_guess)));
    }

    Instrument instr_bm;
    t0 = chrono::high_resolution_clock::now();
    auto [Bp, U_final] = bmssp(G, dist_bm, E, l,
                               numeric_limits<double>::infinity(),
                               {source}, n_nodes, &instr_bm);
    t1 = chrono::high_resolution_clock::now();
    double time_bm = chrono::duration<double>(t1 - t0).count();

    return {time_dij, time_bm};
}

static GraphType parse_graph_type(const string& s) {
    if (s == "random-m")   return GraphType::RANDOM_M;
    if (s == "er")         return GraphType::ER;
    if (s == "ba")         return GraphType::BA;
    if (s == "ws")         return GraphType::WS;
    if (s == "grid2d")     return GraphType::GRID2D;
    if (s == "layered-dag")return GraphType::LAYERED_DAG;
    return GraphType::RANDOM_M;
}

int main(int argc, char* argv[]) {
    // valores por defecto
    int n = 200000, m = 800000;
    unsigned seed0 = 0;
    int trials = 100;
    Node source = 0;
    double wmax = 100.0;
    string out_path = "benchmark_times.csv";
    string gtype_str = "random-m";

    // parámetros específicos
    double p = 0.0005;
    int attach = 2;
    int k = 6; 
    double beta = 0.1;
    int rows = 200, cols = 200; 
    bool diag = false;
    int layers = 50, width = 4000; 
    double dagp = 0.02;

    // parseo de argumentos CLI
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        auto need = [&](int more){ return i + more < argc; };
        if ((a == "-n" || a == "--nodes") && need(1)) n = atoi(argv[++i]);
        else if ((a == "-m" || a == "--edges") && need(1)) m = atoi(argv[++i]);
        else if ((a == "-s" || a == "--seed") && need(1)) seed0 = atoi(argv[++i]);
        else if ((a == "-t" || a == "--trials") && need(1)) trials = atoi(argv[++i]);
        else if ((a == "-o" || a == "--output") && need(1)) out_path = argv[++i];
        else if ((a == "--source") && need(1)) source = static_cast<Node>(atoi(argv[++i]));
        else if ((a == "--wmax") && need(1)) wmax = atof(argv[++i]);
        else if ((a == "--graph") && need(1)) gtype_str = argv[++i];
        // específicos
        else if ((a == "--p") && need(1)) p = atof(argv[++i]);
        else if ((a == "--attach") && need(1)) attach = atoi(argv[++i]);
        else if ((a == "--k") && need(1)) k = atoi(argv[++i]);
        else if ((a == "--beta") && need(1)) beta = atof(argv[++i]);
        else if ((a == "--rows") && need(1)) rows = atoi(argv[++i]);
        else if ((a == "--cols") && need(1)) cols = atoi(argv[++i]);
        else if (a == "--diag") diag = true;
        else if ((a == "--layers") && need(1)) layers = atoi(argv[++i]);
        else if ((a == "--width") && need(1)) width = atoi(argv[++i]);
        else if ((a == "--dagp") && need(1)) dagp = atof(argv[++i]);
    }

    GraphType gtype = parse_graph_type(gtype_str);

    ofstream fout(out_path);
    if (!fout) {
        cerr << "Error: cannot open output file: " << out_path << "\n";
        return 1;
    }
    fout << "trial,seed,time_dijkstra,time_bmssp\n";

    for (int i = 0; i < trials; ++i) {
        GraphGenOptions opt;
        opt.wmax = wmax;
        opt.seed = seed0 + static_cast<unsigned>(i);
        switch (gtype) {
            case GraphType::RANDOM_M:  
                opt.n = n; opt.m = m; break;
            case GraphType::ER:        
                opt.n = n; opt.p = p; break;
            case GraphType::BA:        
                opt.n = n; opt.attach = attach; break;
            case GraphType::WS:        
                opt.n = n; opt.k = k; opt.beta = beta; break;
            case GraphType::GRID2D:    
                opt.rows = rows; opt.cols = cols; opt.diag = diag; break;
            case GraphType::LAYERED_DAG: 
                opt.layers = layers; opt.width = width; opt.dagp = dagp; break;
        }

        auto [G, E] = generate_graph(gtype, opt);
        if (!G.count(source)) source = 0;

        BenchResult r = run_benchmark(G, E, source);
        fout << i << "," << opt.seed << "," << r.time_dij << "," << r.time_bm << "\n";
    }

    fout.close();
    cout << "CSV listo (" << trials << " tests) => " << out_path << "\n";
    return 0;
}
