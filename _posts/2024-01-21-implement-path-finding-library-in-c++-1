---
layout: post
title:  "A Path Finding Implementation with Modern C++ (Part1)"
date:   2024-01-21 11:58:00
categories: C++
brief: "About 10 years ago, I implemented my first A* algorithm with C++98 version. Inspiring by Brian Cairl CppCon23 Presentation I try to implement one by myself in modern C++ fashion at my best. Enjoying.."
---

# Inroduction
Given a map and point A & B in this map, you want to figure out the shortest path from point A to point B. How to achieve this task? The map contains lots of obstacles, maybe a river you have to swim across, maybe a mountain you have to climb, and so on.. It's not only the shortest path, it's also the path that takes least efforts. In the computer science, there is a donzen of path finding algorithm includes A*, D* etc. In the first time, I will introduce the classic one - Dijkstra which was invent by Dr. Edsger W. Dijkstra In 1959. For the detail of the algorthim, there are so many on the internet, so I will not introduce here. I just recommand [Estefania Cassingena Navone's article](https://www.freecodecamp.org/news/dijkstras-shortest-path-algorithm-visual-introduction)
. After you have known the basic of the algorithm. Let's start implement one by yourself. 


# Implement
I will break all the implementation to the following parts

- Graph - Implement the data structure of the map
- GraphBuilder - Helper class which contains the utility to construct one 
- SearchContext - Provide all the intermedia data needed in one search procedure
- Algorithm - The main logic of the path finding algorithm

Besides the above, we also have one example，which put all the thing together. The full code is available on my github https://github.com/zhujiawei6000/path_finding. Let's dive to the detail.

![path-finding-demo](img/2024-01-21_1.gif)

## Graph

graph is a common form of a map. The key concept of graph is node(vetex) and edge. In a simple word, the node represent a place on the map, and the edge represent the road connection. Each edge also has a weight, means the distance between two nodes. Here's the code.
```
namespace pf {
using NodeId = int;
struct NodeProperties {
  std::string name;
  ...
};
using CostType = int;
struct EdgeProperties {
  CostType cost;
  ...
};

using Edge = std::pair<NodeId, EdgeProperties>;
```
The node is very straight-forward just id and some custom properties. And the edge defined by std::pair of distination node and edge properties. edge properties includes cost of the edge and others.

Next, we define the graph with node and edge

```
class Graph {
 private:
  friend class GraphBuilder;
  std::multimap<NodeId, Edge> adjacencies_;
  std::map<NodeId, NodeProperties> nodes_;

 public:
  template <typename EdgeVisitorT>
  void ForEachEdge(NodeId key, EdgeVisitorT&& visitor) const {
    auto range = adjacencies_.equal_range(key);
    std::for_each(range.first, range.second,
                  [visitor](const auto& parent_and_edge) mutable {
                    std::apply(visitor, parent_and_edge.second);
                  });
  }
  const NodeProperties& Node(NodeId nid) const { return nodes_.at(nid); }
};
```

  using ```std::multimap<NodeId, Edge> ``` to store the relationship of the nodes - edges. and ```std::map<NodeId, NodeProperties>``` to store all the nodes and its props.
  
We also add ```ForEachEdge(NodeId key, EdgeVisitorT&& visitor)``` function as accessor for the internal data. The function find the adjacencies of the given node and apply visitor to all the adjacencies.

## Search Context
Search context is used in the searching stage. It's contains 2 main parts. One is current available transitions which may changed throughout the searching. Another is visited nodes which used in the algorithm to prevent duplicating and we also store the previous node to backtrace the path when we reach the goal.

We use ```std::priority_queue<Transition, std::vector<Transition>, std::greater<Transition>>``` here instead of std::queue, becasue it supply better performance to draw the shortest transition in the current iteration.


In the ```Reset(NodeId s, NodeId e)``` function, we push the start node to the transition queue for the iteration step in the next section
```

class SearchContext {
    NodeId goal_;
    std::map<NodeId, NodeId> visited_;
    std::priority_queue<Transition, std::vector<Transition>, std::greater<Transition>> transitions_;
public:
    void Reset(NodeId s, NodeId e) {
        goal_ = e;
        visited_.clear();
        while (!transitions_.empty()) transitions_.pop();
        //  Mark the starting vertex with a distance of zero. Designate this vertex as current.
        Enqueue(s, s, 0);
    }
    ...
};
```

## Dijkstra Algorithm

The basic of the algorithm is very simple, here is key points quoted from the other sites
> - Dijkstra's Algorithm basically starts at the node that you choose (the source node) and it analyzes the graph to find the shortest path between that node and all the other nodes in the graph.
> - The algorithm keeps track of the currently known shortest distance from each node to the source node and it updates these values if it finds a shorter path.
> - Once the algorithm has found the shortest path between the source node and another node, that node is marked as "visited" and added to the path.
>- The process continues until all the nodes in the graph have been added to the path. This way, we have a path that connects the source node to all other nodes following the shortest path possible to reach each node.

It takes me a while to implement it right, here is the sample C++ code. Just a while loop and some lines of code. Most of work was done in the sections above

```
std::vector<NodeId> Dijkstra(const Graph& graph, NodeId start,
                                        NodeId end) {
  SearchContext context;
  // Mark the ending vertex with a distance of zero.
  context.Reset(start, end);
  bool solved = false;
  while (context.QueueIsNotEmpty()) {
    // Designate this vertex as current.
    auto [curr_node, prev_node, cost] = context.Dequeue();
    if (context.IsVisited(curr_node)) {
        continue;
    }
    context.MarkVisited(prev_node, curr_node);
    if (context.IsTerminal(curr_node)) {
      solved = true;
      break;
    }
    graph.ForEachEdge(curr_node, [&context, curr_node, cost](
                                     NodeId nid, const EdgeProperties& props) {
      // Don’t record this distance if it is longer than a previously recorded
      // distance.
      if (!context.IsVisited(nid)) {
        context.Enqueue(nid,               // curr node
                        curr_node,         // prev
                        props.cost + cost  // cost
        );
      }
    });
  }
  ```

  ## GraphBuilder

  To make the example work, we should also build a map by ourself. To make the work a little bit earsier, I wrote the GraphBuilder class.

  I make it a friend class of Graph to access its data member without breaking the encapsulating. ```Link(NodeId from, NodeId to, CostType cost)``` responsible for create connection and ``` Node(NodeId id, std::string name="")``` use to create the nodes

  ```
class GraphBuilder {
  Graph& graph_;
public:
  explicit GraphBuilder(Graph& graph) : graph_{graph} {}

  void Link(NodeId from, NodeId to, CostType cost) {
    auto& nodes = graph_.nodes_;
    assert(nodes.find(from) != nodes.end() &&
           nodes.find(to) != nodes.end());
    graph_.adjacencies_.emplace(from, std::make_pair(to, pf::EdgeProperties{cost}));
  };
  void Node(NodeId id, std::string name="") {
    auto& nodes = graph_.nodes_;
    assert(nodes.find(id) == nodes.end());
    nodes.emplace(id, pf::NodeProperties{name});
  }
};
```

## Summary
In this part we implemented the basic path finding algorithm Dijkstras. Although it's not suitable for the production needed due to performance hit, it's still a good start to optimize it further.
