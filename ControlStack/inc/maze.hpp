#include <vector>
#include <random>
#include <algorithm>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::Vector2d;
using Eigen::Vector2i;
using std::vector;
// #define DEBUG

class Maze {
public:
    // Constructor
    Maze(int rows, int cols, double cellWidth, double cellHeight, double wallThickness)
        : rows_(rows + 1), cols_(cols + 1), cellWidth_(cellWidth), cellHeight_(cellHeight), wallThickness_(wallThickness), 
          verticalEdges_(rows, std::vector<bool>(cols + 1, false)), 
          horizontalEdges_(rows + 1, std::vector<bool>(cols, false)) {
        generateMaze();
    }

    struct Node {
        int r;
        int c;
        Vector2i move;

        Node(int row, int col, Vector2i mv) : r(row), c(col), move(mv) {}
    };

    // Method to generate a random maze
    void generateMaze() {
        num_visited_ = 0;
        // Set all walls true
        for (auto& row: verticalEdges_){
            std::fill(row.begin(), row.end(), true);
        }
        for (auto& row: horizontalEdges_){
            std::fill(row.begin(), row.end(), true);
        }
        dfs(0, 0);
    }

    bool isLegal(Matrix<bool, Dynamic, Dynamic> &visited, const int r, const int c, const Vector2i move) {
        const int r_next = r + move(0);
        const int c_next = c + move(1);

        if (r_next >= 0 && r_next < visited.rows() && c_next >= 0 && c_next < visited.cols()) {
            return !visited(r_next, c_next);
        } else {
            return false;
        }               
    }

    std::vector<Vector2i> getMoves(Matrix<bool, Dynamic, Dynamic> &visited, const int r, const int c) {
        // List possible moves
        std::vector<Vector2i> moves = {Vector2i(-1, 0), Vector2i(1, 0), Vector2i(0, -1), Vector2i(0, 1)};

        // Filter out illegal moves
        moves.erase(
            std::remove_if(moves.begin(), moves.end(), [&](const Vector2i move) {return !isLegal(visited, r, c, move);}),
            moves.end()
        );
        // Shuffle the moves
        if (moves.size() > 0) {
            std::random_device rd;                          // Obtain a random number from hardware
            std::mt19937 gen(rd());                         // Seed the generator
            std::shuffle(moves.begin(), moves.end(), gen);  // Shuffle legals moves
        }
        return moves;
    }

    void dfs(const int r, const int c) {
        Matrix<bool, Dynamic, Dynamic> visited;
        visited.resize(rows_ - 1, cols_ - 1);
        visited.setConstant(false);

        std::vector<Node> stack;

        stack.push_back(Node(r, c, Vector2i(0, 0)));

        while (!stack.empty()) {
            Node current = stack.back();
            stack.pop_back();

            const int r = current.r;
            const int c = current.c;
            const Vector2i move = current.move;
            const int r_next = r + move(0);
            const int c_next = c + move(1);

            if (visited(r_next, c_next)) {
                continue;
            } else {
                #ifdef DEBUG
                    std::cout << "\nMove: (" << r << ", " << c << ") to (" << r_next << ", " << c_next << ")";
                    #endif
                // Cut edge along move
                if (move(0) == 0 && move(1) != 0) { // Horizontal move, cut vertical edge
                    int c_cut = move(1) > 0 ? c + move(1) : c;
                    #ifdef DEBUG
                    std::cout << "\tCut V: " << r << ", " << c_cut << std::endl;
                    #endif
                    verticalEdges_[r][c_cut] = false;
                } else if (move(0) != 0 && move(1) == 0) { // Vertical move, cut horizontal edge
                    int r_cut = move(0) > 0 ? r + move(0) : r;
                    #ifdef DEBUG
                    std::cout << "\tCut H: " << r_cut << ", " << c << std::endl;
                    #endif
                    horizontalEdges_[r_cut][c] = false;
                }
            }
            visited(r_next, c_next) = true;

            std::vector<Vector2i> moves = getMoves(visited, r_next, c_next);
            for (auto m : moves) {
                stack.push_back(Node(r_next, c_next, m));
            }
        }
    }

    // Corrected method definition for computing wall vertices
    vector<MatrixXd> computeWallVertices() const {
        vector<MatrixXd> obstacles;

        // Compute vertical walls
        for (int j = 0; j < cols_; ++j) {
            int i = 0;
            while (i < rows_ - 1) {
                if (verticalEdges_[i][j]) {
                    double x = j * cellWidth_;
                    double y = i * cellHeight_;
                    MatrixXd rect(4, 2); // 4x2 matrix for rectangle vertices
                    rect << x - wallThickness_ / 2, y,
                            x - wallThickness_ / 2, y + cellHeight_,
                            x + wallThickness_ / 2, y + cellHeight_,
                            x + wallThickness_ / 2, y;
                    while (i < rows_ - 1 && verticalEdges_[i][j]) {
                        double y = i * cellHeight_;
                        rect(1, 1) = y + cellHeight_;
                        rect(2, 1) = y + cellHeight_;
                        i++;
                    }
                    obstacles.push_back(rect);
                }
                i++;
            }
        }


        for (int i = 0; i < rows_; ++i) {
            int j = 0;
            while (j < cols_ - 1) {
                if (horizontalEdges_[i][j]) {
                    double x = j * cellWidth_;
                    double y = i * cellHeight_;
                    MatrixXd rect(4, 2); // 4x2 matrix for rectangle vertices
                    rect << x - wallThickness_ / 2, y - wallThickness_ / 2,
                            x - wallThickness_ / 2, y + wallThickness_ / 2,
                            x + wallThickness_ / 2 + cellWidth_, y + wallThickness_ / 2,
                            x + wallThickness_ / 2 + cellWidth_, y - wallThickness_ / 2;
                    while (j < cols_ - 1 && horizontalEdges_[i][j]) {
                        double x = j * cellWidth_;
                        rect(2, 0) = x + wallThickness_ / 2 + cellWidth_;
                        rect(3, 0) = x + wallThickness_ / 2 + cellWidth_;
                        j++;
                    }
                    obstacles.push_back(rect);
                }
                j++;
            }
        }

        return obstacles;
    }

private:
    int num_visited_;
    int rows_;
    int cols_;
    double cellWidth_;
    double cellHeight_;
    double wallThickness_;
    std::vector<std::vector<bool>> verticalEdges_;  // (rows x (cols - 1))
    std::vector<std::vector<bool>> horizontalEdges_; // ((rows - 1) x cols)
};