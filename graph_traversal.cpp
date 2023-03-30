#include "graph.h"
struct Settings {
    bool verbose{false};
    int start {86771};
    int end {110636};
    std::string algo{"bfs"};
    std::string file{""};
};

typedef std::function<void(Settings&)> NoArgHandle;

#define S(str, f, v) {str, [](Settings& s) {s.f = v;}}
const std::unordered_map<std::string, NoArgHandle> NoArgs {
  S("--verbose", verbose, true),
  S("-v", verbose, true),
};
#undef S

typedef std::function<void(Settings&, const std::string&)> OneArgHandle;

#define S(str, f, v) \
  {str, [](Settings& s, const std::string& arg) { s.f = v; }}

const std::unordered_map<std::string, OneArgHandle> OneArgs {
  // Writing out the whole lambda
  {"-f", [](Settings& s, const std::string& arg) {
    s.file = arg;
  }},

  // Using the macro
  S("--file", file, arg),
  S("--start", start,  stoi(arg)),
  S("--end", end,  stoi(arg)),
  S("--algorithm", algo, arg)
};
#undef S

Settings parse_settings(int argc, const char* argv[]) {
  Settings settings;

  // argv[0] is traditionally the program name, so start at 1
  for(int i {1}; i < argc; i++) {
    std::string opt {argv[i]};

    // Is this a NoArg?
    if(auto j {NoArgs.find(opt)}; j != NoArgs.end())
      j->second(settings); // Yes, handle it!

    // No, how about a OneArg?
    else if(auto k {OneArgs.find(opt)}; k != OneArgs.end())
      // Yes, do we have a parameter?
      if(++i < argc)
        // Yes, handle it!
        k->second(settings, {argv[i]});
      else
        // No, and we cannot continue, throw an error
        throw std::runtime_error {"missing param after " + opt};

    // No, has infile been set yet?
    else if(settings.file.empty())
      // No, use this as the input file
      settings.file = argv[i];

    // Yes, possibly throw here, or just print an error
    else
      std::cerr << "unrecognized command-line option " << opt << std::endl;
  }

  return settings;
}

enum Algorithm {
    bfs,
    dijkstra,
    astar,
    dijkstra_priority,
    astar_priority
};

static const char *algoStr[] = { "bfs", "dijkstra", "astar", "dijkstra_priority", "astar_priority"};

class Commify {
private:
    std::string str_;

public:
    explicit Commify(int64_t value) {
        str_ = std::to_string(value);
        int len = str_.length();
        for (int i = len - 3; i > 0; i -= 3) {
            str_.insert(i, ",");
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const Commify& c) {
        os << c.str_;
        return os;
    }
};


int main(int argc, const char *argv[])
{

    // graph_traversal --start 86771 --end 110636 --algorithm bfs --file graph_dc_area.2022-03-11.txt
    if (argc != 9)
    {
        std::cout << "Not enough arguments specified, command must look like ./map_path_finder --start 86771 --end 110636 --algorithm bfs --file graph_dc_area.2022-03-11.txt" << std::endl;
        return 0;
    }
    Settings settings = parse_settings(argc, argv);
    // create graph
    Graph g(settings.file);

    std::vector<std::pair<int, double>>  path;
    auto start = std::chrono::high_resolution_clock::now();
    if (settings.algo == algoStr[Algorithm::bfs])
    {
        path = g.bfs(settings.start, settings.end);

    }
    else if (settings.algo == algoStr[Algorithm::dijkstra])
    {
        path = g.dijkstra(settings.start, settings.end);

    }
    else if (settings.algo == algoStr[Algorithm::astar])
    {
        path = g.astar(settings.start, settings.end);
    }
    else if (settings.algo == algoStr[Algorithm::dijkstra_priority])
    {
        path = g.astar(settings.start, settings.end);
    }
    else if (settings.algo == algoStr[Algorithm::astar_priority])
    {
        path = g.dijkstra(settings.start, settings.end);

    }
    auto stop = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    g.printPath(path);

    std::cout << "Info: path calculated in " << Commify(duration.count()) << "us" << std::endl;
    
    return 1;
}
