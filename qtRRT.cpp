#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <QApplication>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>

using namespace std;

// Define a struct to represent a node in the RRT
struct Node {
    double x, y;
    int parent;
};

// Define a struct to represent an obstacle
struct Obstacle {
    double x, y, r;
};

// Define a function to check if a point is inside an obstacle
bool insideObstacle(double x, double y, vector<Obstacle> obstacles) {
    for (int i = 0; i < obstacles.size(); i++) {
        double distToObstacle = sqrt(pow(x - obstacles[i].x, 2) + pow(y - obstacles[i].y, 2));
        if (distToObstacle < obstacles[i].r) {
            return true;
        }
    }
    return false;
}

// Define a function to check if a line segment collides with an obstacle
bool lineCollision(double x1, double y1, double x2, double y2, vector<Obstacle> obstacles) {
    for (int i = 0; i < obstacles.size(); i++) {
        double distToObstacle = sqrt(pow(obstacles[i].x - x1, 2) + pow(obstacles[i].y - y1, 2));
        if (distToObstacle < obstacles[i].r) {
            return true;
        }
        double dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        double dx = (x2 - x1) / dist;
        double dy = (y2 - y1) / dist;
        double t = 0;
        while (t < dist) {
            t += 0.1;
            double x = x1 + t * dx;
            double y = y1 + t * dy;
            distToObstacle = sqrt(pow(obstacles[i].x - x, 2) + pow(obstacles[i].y - y, 2));
            if (distToObstacle < obstacles[i].r) {
                return true;
            }
        }
    }
    return false;
}

// Define a function to generate a random point within the workspace
Node randomPoint(double minX, double minY, double maxX, double maxY) {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> xDist(minX, maxX);
    uniform_real_distribution<double> yDist(minY, maxY);
    Node node;
    node.x = xDist(gen);
    node.y = yDist(gen);
    node.parent = -1;
    return node;
}

// Define a function to find the nearest neighbor in the RRT
int nearestNeighbor(Node node, vector<Node> tree) {
    int nearest = 0;
    double minDist = sqrt(pow(node.x - tree[0].x, 2) + pow(node.y - tree[0].y, 2));
    for (int i = 1; i < tree.size(); i++) {
        double dist = sqrt(pow(node.x - tree[i].x, 2) + pow(node.y - tree[i].y, 2));
        if (dist < minDist) {
            nearest = i;
            minDist = dist;
        }
    }
    return nearest;
}

// Define a function to generate a new node towards a random point
Node steer(Node start, Node end, double stepSize)
{
    double dist = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
    if (dist <= stepSize) {
        return end;
    } else {
        double theta = atan2(end.y - start.y, end.x - start.x);
        Node node;
        node.x = start.x + stepSize * cos(theta);
        node.y = start.y + stepSize * sin(theta);
        node.parent = start.parent;
        return node;
    }
}

// Define a function to check if the goal is reachable from the current node
bool isGoalReachable(Node node, Node goal, vector<Obstacle> obstacles, double stepSize) {
    double dist = sqrt(pow(node.x - goal.x, 2) + pow(node.y - goal.y, 2));
    if (dist <= stepSize) {
        return !lineCollision(node.x, node.y, goal.x, goal.y, obstacles);
    } else {
        Node intermediate = steer(node, goal, stepSize);
        return !lineCollision(node.x, node.y, intermediate.x, intermediate.y, obstacles) &&
               isGoalReachable(intermediate, goal, obstacles, stepSize);
    }
}

// Define the RRT algorithm function
vector<Node> rrt(Node start, Node goal, vector<Obstacle> obstacles, double minX, double minY, double maxX, double maxY, double stepSize, int maxIterations) {
    vector<Node> tree;
    tree.push_back(start);
    for (int i = 0; i < maxIterations; i++) {
        Node rand = randomPoint(minX, minY, maxX, maxY);
        int nearest = nearestNeighbor(rand, tree);
        Node newnode = steer(tree[nearest], rand, stepSize);
        if (!lineCollision(tree[nearest].x, tree[nearest].y, newnode.x, newnode.y, obstacles)) {
            if (isGoalReachable(newnode, goal, obstacles, stepSize)) {
                Node goalNode = goal;
                goalNode.parent = tree.size();
                tree.push_back(newnode);
                return tree;
            } else {
                newnode.parent = nearest;
                tree.push_back(newnode);
            }
        }
    }
    return tree;
}

// Define a function to find the path from the RRT
vector<Node> findPath(vector<Node> tree, Node goal) {
    vector<Node> path;
    if (tree.size() > 0) {
        path.push_back(goal);
        int parent = goal.parent;
        while (parent != -1) {
            path.push_back(tree[parent]);
            parent = tree[parent].parent;
        }
    }
    return path;
}

// Define a function to draw a node on the graphics scene
void drawNode(QGraphicsScene* scene, Node node, QBrush brush) {
    QGraphicsEllipseItem* item = scene->addEllipse(node.x - 2, node.y - 2, 4, 4);
    item->setBrush(brush);
}

// Define a function to draw a line segment on the graphics scene
void drawLine(QGraphicsScene* scene, double x1, double y1, double x2, double y2, QPen pen) {
    QGraphicsLineItem* item = scene->addLine(x1, y1, x2, y2, pen);
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    QMainWindow window;
    QGraphicsScene* scene = new QGraphicsScene();
    QGraphicsView* view = new QGraphicsView(scene);
    window.setCentralWidget(view);
    window.resize(1200, 900);
    // Define the start and goal nodes
    Node start = {50, 50, -1};
    Node goal = {700, 500, -1};

    // Define the obstacles
    vector<Obstacle> obstacles;
    Obstacle obs1 = {{200, 200, 10}, {300, 200, 10}, {300, 300, 10}, {200, 300, 10}};
    obstacles.push_back(obs1);
    Obstacle obs2 = {{400, 400,10}, {500, 400,10}, {500, 500,10}, {400, 500,10}};
    obstacles.push_back(obs2);

    // Define the parameters for the RRT algorithm
    double minX = 0, minY = 0, maxX = 1200, maxY = 900, stepSize = 20;
    int maxIterations = 10000;

    // Run the RRT algorithm
    vector<Node> tree = rrt(start, goal, obstacles, minX, minY, maxX, maxY, stepSize, maxIterations);

    // Find the path from the RRT
    vector<Node> path = findPath(tree, goal);

    // Draw the obstacles on the graphics scene
    for (int i = 0; i < obstacles.size(); i++) {
        drawObstacle(scene, obstacles[i]);
    }

    // Draw the nodes and edges of the RRT
    for (int i = 0; i < tree.size(); i++) {
        if (tree[i].parent != -1) {
            drawLine(scene, tree[i].x, tree[i].y, tree[tree[i].parent].x, tree[tree[i].parent].y, QPen(Qt::black));
        }
        drawNode(scene, tree[i], QBrush(Qt::blue));
    }

    // Draw the path on the graphics scene
    for (int i = 0; i < path.size() - 1; i++) {
        drawLine(scene, path[i].x, path[i].y, path[i+1].x, path[i+1].y, QPen(Qt::red, 3));
    }

    // Show the graphics scene
    view->setSceneRect(0, 0, maxX, maxY);
    view->show();
    return app.exec();
}