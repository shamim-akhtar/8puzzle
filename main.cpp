#include <vector>
#include <random>
#include <map>
#include <iostream>
#include <cassert>

//! A typedef of a normal integer array using 
//std::vector for convenience
typedef std::vector<int> IntArray;

///class State
///A class to hold the state of the puzzle. 
///The state is represented by a simple one dimensional array of integers.
///The value of o represents empty slot.
class State
{
public:
  ///
  explicit State(unsigned int rows_or_cols)
    : _rows_or_cols(rows_or_cols)
  {
    _state.resize(_rows_or_cols * _rows_or_cols);
    for (unsigned int i = 0; i < _state.size(); ++i)
    {
      _state[i] = i;
    }
  }

  State(unsigned int rows_or_cols, const IntArray& arr)
    : _rows_or_cols(rows_or_cols)
  {
    assert(arr.size() == _rows_or_cols * _rows_or_cols);
    _state = arr;
  }

  ///copy constructor
  State(const State& other)
  {
    _rows_or_cols = other._rows_or_cols;
    _state = other._state;
  }

  ///assignment operator
  State& operator = (const State& other)
  {
    if (this != &other)
    {
      _rows_or_cols = other._rows_or_cols;
      _state = other._state;
    }
    return *this;
  }

  inline const IntArray& getArray() const
  {
    return _state;
  }

  void setArray(const IntArray& arr)
  {
    _state = arr;;
  }

  ///equal to operator. This will check item by item.
  friend bool operator == (const State& a, const State& b)
  {
    return (a._state == b._state);
  }
  /// find the index of the empty slot
  inline int findZeroIndex() const
  {
    for (int i = 0; i < (int)_state.size(); ++i)
      if (_state[i] == 0) return i;

    return (int)_state.size();
  }
  /// randomize teh state. 
  ///NOTE: Not all randomized states are solvable. 
  ///Need to implement a method to find whether a state is solvable or not.
  inline void randomize()
  {
    std::random_shuffle(_state.begin(), _state.end());
  }

  ///swap the values of the indices
  inline void swap_index(int i0, int i1)
  {
    int tmp = _state[i1];
    _state[i1] = _state[i0];
    _state[i0] = tmp;
  }

  inline int gethammingCost()
  {
    int cost = 0;
    for (unsigned int i = 0; i < _state.size(); ++i)
    {
      if (_state[i] == 0) continue;
      if (_state[i] != i + 1) cost += 1;
    }
    return cost;
  }

  inline int getManhattanCost()
  {
    int cost = 0;
    for (unsigned int i = 0; i < _state.size(); ++i)
    {
      int v = _state[i];
      if (v == 0) continue;

      // actual index of v should be v-1
      v = v - 1;
      int gx = v % _rows_or_cols;
      int gy = v / _rows_or_cols;

      int x = i % _rows_or_cols;
      int y = i / _rows_or_cols;

      int mancost = abs(x - gx) + abs(y - gy);
      cost += mancost;

      int z = 0;
    }
    return cost;
  }

  void print(std::ostream& str, bool flat = false) const
  {
    for (unsigned int i = 0; i < _rows_or_cols; ++i)
    {
      for (unsigned int j = 0; j < _rows_or_cols; ++j)
      {
        unsigned int index = i * _rows_or_cols + j;
        if (flat)
        {
          str << _state[index];
        }
        else
        {
          str << _state[index] << " ";
        }
      }
      if (!flat)
      {
        str << "\n";
      }
    }
    str << "\n";
  }

private:
  IntArray _state;
  unsigned int _rows_or_cols;
};

class Node
{
public:
  Node(const State& state, std::shared_ptr<Node> parent, int depth = 0)
    : _state(state)
    , _depth(depth)
  {
    _parent = parent;
    _mc = _state.getManhattanCost();
    _hc = _state.gethammingCost();
  }

  inline int getManhattanCost() const
  {
    return _mc;
  }

  inline int getHemmingCost() const
  {
    return _hc;
  }

  void setParent(Node* node)
  {
    _parent.reset(node);
  }

  void setParent(std::shared_ptr<Node> node)
  {
    _parent = node;
  }

  std::shared_ptr<Node> getParent()
  {
    return _parent;
  }

  const std::shared_ptr<Node> getParent() const
  {
    return _parent;
  }

  const State& getState() const
  {
    return _state;
  }

  int getDepth() const
  {
    return _depth;
  }

  void print(std::ostream& out, int lineNum) const
  {
    /*for (int k = 0; k < _depth; ++k) out << " ";*/
    out << lineNum << " - Node { ";
    for (unsigned int i = 0; i < _state.getArray().size(); ++i)
    {
      out << _state.getArray()[i];
    }
    out << " | D: " << _depth << ", MD: " << _mc;
    out << " }" << "\n";
  }

private:
  State _state;
  std::shared_ptr<Node> _parent;

  int _mc;
  int _hc;
  int _depth;
};

typedef std::shared_ptr<Node> NodePtr;

class Neighbours
{
public:
  typedef std::map<int, std::vector<int> > IndexNeighbourMap;
  IndexNeighbourMap _edges;

  Neighbours()
  {
    CreateGraphFor8Puzzle();
  }

  const std::vector<int>& getNeighbors(int id) const
  {
    IndexNeighbourMap::const_iterator itr(_edges.find(id));
    if (itr != _edges.end()) return itr->second;
    static std::vector<int> s;
    return s;
  }

private:
  void CreateGraphFor8Puzzle()
  {
    /*
    0 1 2
    3 4 5
    6 7 8
    */
    _edges.insert(std::make_pair(0, std::vector<int>{ 1, 3 }));
    _edges.insert(std::make_pair(1, std::vector<int>{ 0, 2, 4 }));
    _edges.insert(std::make_pair(2, std::vector<int>{ 1, 5 }));
    _edges.insert(std::make_pair(3, std::vector<int>{ 4, 0, 6 }));
    _edges.insert(std::make_pair(4, std::vector<int>{ 3, 5, 1, 7 }));
    _edges.insert(std::make_pair(5, std::vector<int>{ 4, 2, 8 }));
    _edges.insert(std::make_pair(6, std::vector<int>{ 7, 3 }));
    _edges.insert(std::make_pair(7, std::vector<int>{ 6, 8, 4 }));
    _edges.insert(std::make_pair(8, std::vector<int>{ 7, 5 }));
  }
};

inline int random_range(int min, int max)
{
  //int output = min + (rand() * (int)(max - min) / RAND_MAX);
  //return output;

  static std::random_device rd; // only used once to initialise (seed) engine
  std::mt19937 rng(rd()); // random-number engine used (Mersenne-Twister in this case)
  std::uniform_int_distribution<int> uni(min, max); // guaranteed unbiased

  auto random_integer = uni(rng);
  return random_integer;
}

///Helper function that enables the creation of a random solvable state.
inline void createRandomSolvableState(unsigned int depth,
  const Neighbours& graph, State& state)
{
  for (unsigned int i = 0; i < depth; ++i)
  {
    int zero = state.findZeroIndex();
    const IntArray& neighbours = graph.getNeighbors(zero);
    // get a random neignbour.
    int index = random_range(0, (int)neighbours.size() - 1);
    state.swap_index(zero, neighbours[index]);
    //state.print(std::cout);
  }
}

class CompareFunctorForGreedyBestFirst
{
public:
  bool operator()(
    const std::shared_ptr< Node>& n1,
    const std::shared_ptr< Node>& n2) const
  {
    return (n1->getManhattanCost() + n1->getHemmingCost()) <
      (n2->getManhattanCost() + n2->getHemmingCost());
  }
};

class CompareFunctorForAStar
{
public:
  bool operator()(
    const std::shared_ptr< Node>& n1,
    const std::shared_ptr< Node>& n2) const
  {
    return (n1->getManhattanCost() + n1->getDepth() + n1->getHemmingCost()) <
      (n2->getManhattanCost() + n2->getDepth() + n2->getHemmingCost());
  }
};

inline bool isInArray(const State& state, const std::vector<std::shared_ptr<Node> >& li)
{
  unsigned int i = 0;
  for (; i < li.size(); ++i)
  {
    if (state == li[i]->getState())
      return true;
  }
  return false;
}

class Solver
{
public:
  enum Type
  {
    DEPTH_FIRST = 0,
    BREADTH_FIRST,
    GREEDY_BEST_FIRST,
    ASTAR,
  };

  Solver(const State& start, const State& goal)
    : _goal(goal)
    , _solved(false)
  {
    NodePtr root(new Node(start, 0, 0));
    _openlist.push_back(root);
    //_closedlist.push_back(root);
  }

  virtual ~Solver()
  {
  }

  inline bool isSolved() const
  {
    return _solved;
  }

  inline bool isSolvable() const
  {
    ///TODO
    return true;
  }

  ///Returns next node in the search.
  //template osg::ref_ptr GetNextNode(Compare cmp)
  NodePtr GetNextNode(Type type = Type::ASTAR)
  {
    if (_openlist.empty()) return 0;
    NodePtr current;

    switch (type)
    {
    case ASTAR:
    {
      NodeList::iterator current_itr(
        std::min_element(_openlist.begin(), _openlist.end(),
          CompareFunctorForAStar())
      );

      if (current_itr == _openlist.end()) return 0;

      //copy the value first to a shared pointer and then 
                  //erase from the open list.
      current = *current_itr;

      // now erase from the open list.
      _openlist.erase(current_itr);
      _closedlist.push_back(current);

      break;
    }
    case GREEDY_BEST_FIRST:
    {
      NodeList::iterator current_itr(
        std::min_element(_openlist.begin(), _openlist.end(),
          CompareFunctorForGreedyBestFirst())
      );

      if (current_itr == _openlist.end()) return 0;

      //copy the value first to a shared pointer and then 
                  //erase from the open list.
      current = *current_itr;

      // now erase from the open list.
      _openlist.erase(current_itr);
      _closedlist.push_back(current);

      break;
    }
    case BREADTH_FIRST:
    {
      current = _openlist[0];
      _openlist.erase(_openlist.begin());
      _closedlist.push_back(current);

      break;
    }
    case DEPTH_FIRST:
      std::cout << "DEPTH_FIRST not implemented\n";
      break;
    }
    return current;
  }

  // expand the graph by looking into the neighbours for the given node.
  void ExpandNode(NodePtr current, const Neighbours& graph)
  {
    if (current->getState() == _goal)
    {
      _solved = true;
      return;
    }

    int zero = current->getState().findZeroIndex();
    const IntArray& neighbours = graph.getNeighbors(zero);

    for (int next : neighbours)
    {
      State state = current->getState();
      state.swap_index(zero, next);

      if (!isInArray(state, _closedlist))
      {
        NodePtr n(new Node(state, current, current->getDepth() + 1));
        _openlist.push_back(n);
        static int s_lineNum = 1;
        n->print(std::cout, s_lineNum++);
        //_closedlist.push_back(n);
      }
    }
  }

private:
  typedef std::vector<std::shared_ptr<Node> > NodeList;
  NodeList _openlist;
  NodeList _closedlist;
  const State& _goal;
  bool _solved;
};

int main(int argc, char* argv[])
{
  Neighbours g;

  State goal(3, std::vector<int>{ 1, 2, 3, 4, 5, 6, 7, 8, 0 });
  //State start(3, std::vector{ 1, 6, 2, 0, 4, 3, 7, 5, 8 });

  State start(3, std::vector<int>{ 3, 7, 8, 2, 0, 6, 4, 5, 1 });

  std::shared_ptr<Node> node;
  Solver solver(start, goal);
  if (!solver.isSolvable())
  {
    std::cout << "Puzzle state is unsolvable..!\n";
    return 0;
  }
  int count = 0;
  while (!solver.isSolved())
  {
    node = solver.GetNextNode(Solver::ASTAR);
    solver.ExpandNode(node, g);
    count++;
  }

  // accumulate the nodes for the solution.
  std::vector<std::shared_ptr<Node> > solution;
  NodePtr s = node;
  do
  {
    solution.push_back(s);
    s = s->getParent();
  } while (s != NULL);

  // print the solution.
  std::cout << "The puzle can be solved in " << solution.size() - 1
    << " steps. Solution below\n";
  for (int i = (int)solution.size() - 1; i >= 0; i--)
  {
    solution[i]->getState().print(std::cout, false);
  }
  std::cout << "\n";

  return 0;
}