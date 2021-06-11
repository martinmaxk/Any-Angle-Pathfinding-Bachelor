# Any-Angle-Pathfinding-Bachelor
Benchmark various Any-Angle-Pathfinding algorithms on pre-made maps created for the purpose.

This includes A* on Visibility Graphs (reduced and not), Theta* and 8-directional A* with and without post-smoothing on grids.

This implementation supports rectangular characters on grids and polygonal characters with a Visibility Graph.
On Visibility Graphs, obstacles are expanded by using triangulation and Minkowski sums and the union of these polygons is found before building the Visibility graph.

To perform benchmarking, in the terminal execute 
```console
dotnet run ExperimentXMap --configuration Release
``` 
where "X" is map 1-5.
