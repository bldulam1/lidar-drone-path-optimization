from collections import deque, namedtuple

inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')


def make_edge(start, end, cost=1):
    return Edge(start, end, cost)


class Graph:
    def __init__(self, edges):
        # let's check that the data is right
        wrong_edges = [i for i in edges if len(i) not in [2, 3]]
        if wrong_edges:
            raise ValueError('Wrong edges data: {}'.format(wrong_edges))

        self.edges = [make_edge(*edge) for edge in edges]
        self.previous_vertices = None
        self.source = None

    @property
    def vertices(self):
        return set(
            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def get_node_pairs(self, n1, n2, both_ends=True):
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def get_previous_vertices(self, source):
        if self.previous_vertices is None or self.source != source:
            distances = {vertex: inf for vertex in self.vertices}
            self.previous_vertices = {
                vertex: None for vertex in self.vertices
            }
            distances[source] = 0
            vertices = self.vertices.copy()

            while vertices:
                current_vertex = min(vertices, key=lambda vertex: distances[vertex])
                vertices.remove(current_vertex)
                if distances[current_vertex] == inf:
                    break
                for neighbour, cost in self.neighbours[current_vertex]:
                    alternative_route = distances[current_vertex] + cost
                    if alternative_route < distances[neighbour]:
                        distances[neighbour] = alternative_route
                        self.previous_vertices[neighbour] = current_vertex

        return self.previous_vertices

    def shortest_path(self, source, dest):
        assert source in self.vertices, 'Such source node doesn\'t exist'

        pv = self.get_previous_vertices(source)
        path, current_vertex = deque(), dest
        while pv[current_vertex] is not None:
            path.appendleft(current_vertex)
            current_vertex = pv[current_vertex]
        if path:
            path.appendleft(current_vertex)
        return path

    def shortest_path_traverse_all(self, source, visited=None, path=None, distance=0, paths=None):
        if paths is None:
            paths = []
        if visited is None:
            visited = set()
        if path is None:
            path = []

        visited.add(source)
        for w, c in self.neighbours[source]:
            if w not in visited:
                visited.add(w)
                p = self.shortest_path_traverse_all(
                    source=w, visited=visited, path=path + [source],
                    distance=distance + c, paths=paths
                )
                if len(self.vertices) - len(visited) <= 1:
                    paths.append((p, distance + c))
                visited.remove(w)

        return path + [source]

    # def remove_edge(self, n1, n2, both_ends=True):
    #     node_pairs = self.get_node_pairs(n1, n2, both_ends)
    #     edges = self.edges[:]
    #     for edge in edges:
    #         if [edge.start, edge.end] in node_pairs:
    #             self.edges.remove(edge)
    #
    # def add_edge(self, n1, n2, cost=1, both_ends=True):
    #     node_pairs = self.get_node_pairs(n1, n2, both_ends)
    #     for edge in self.edges:
    #         if [edge.start, edge.end] in node_pairs:
    #             return ValueError('Edge {} {} already exists'.format(n1, n2))
    #
    #     self.edges.append(Edge(start=n1, end=n2, cost=cost))
    #     if both_ends:
    #         self.edges.append(Edge(start=n2, end=n1, cost=cost))


if __name__ == '__main__':
    graph = Graph([
        ("a", "b", 7), ("a", "c", 9), ("a", "f", 14), ("b", "c", 10),
        ("b", "d", 15), ("c", "d", 11), ("c", "f", 2), ("d", "e", 6),
        ("e", "f", 9)
    ])

    # graph.bfs(start="a")
    p = []
    graph.shortest_path_traverse_all("a", paths=p)
    print(p)
