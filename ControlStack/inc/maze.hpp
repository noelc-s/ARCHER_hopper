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

class Maze {
public:
    // Constructor
    Maze(int rows, int cols, double cellWidth, double cellHeight, double wallThickness)
        : rows_(rows), cols_(cols), cellWidth_(cellWidth), cellHeight_(cellHeight), wallThickness_(wallThickness), 
          verticalEdges_(rows - 1, std::vector<bool>(cols, false)), 
          horizontalEdges_(rows, std::vector<bool>(cols - 1, false)) {
        generateMaze();
    }

    // Method to generate a random maze
    void generateMaze() {
        // Set all walls true
        for (auto& row: verticalEdges_){
            std::fill(row.begin(), row.end(), true);
        }
        for (auto& row: horizontalEdges_){
            std::fill(row.begin(), row.end(), true);
        }
        Matrix<bool, Dynamic, Dynamic> visited;
        visited.resize(rows_ - 1, cols_ - 1);
        visited.setConstant(false);
        dfs(visited, 0, 0, 0, 0);
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

    void dfs(Matrix<bool, Dynamic, Dynamic> &visited, const int r, const int c, const int m_r, const int m_c) {
        const int r_next = r + m_r;
        const int c_next = c + m_c;
        // If visited, return
        if (visited(r_next, c_next)) {
            return;
        } else {
            // Cut edge along move
            if (m_r == 0 && m_c != 0) { // Horizontal move, cut vertical edge
                int c_cut = m_c > 0 ? c + m_c : c;
                verticalEdges_[r][c_cut] = false;
            } else if (m_r != 0 && m_c == 0) { // Vertical move, cut horizontal edge
                int r_cut = m_r > 0 ? r + m_r : r;
                horizontalEdges_[r_cut][c] = false;
            }
        }
        visited(r_next, c_next) = true;

        // List possible moves
        std::vector<Vector2i> moves = {Vector2i(-1, 0), Vector2i(1, 0), Vector2i(0, -1), Vector2i(0, 1)};

        // Filter out illegal moves
        moves.erase(
            std::remove_if(moves.begin(), moves.end(), [&](const Vector2i move) {return !isLegal(visited, r_next, c_next, move);}),
            moves.end()
        );
        
        if (moves.size() > 0) {
            std::random_device rd;                          // Obtain a random number from hardware
            std::mt19937 gen(rd());                         // Seed the generator
            std::shuffle(moves.begin(), moves.end(), gen);  // Shuffle legals moves

            for (auto& move : moves) {
                dfs(visited, r_next, c_next, move(0), move(1));
            }
        }
    }

    // Corrected method definition for computing wall vertices
    vector<MatrixXd> computeWallVertices() const {
        vector<MatrixXd> obstacles;

        // Compute vertical walls
        for (int i = 0; i < rows_ - 1; ++i) {
            for (int j = 0; j < cols_; ++j) {
                if (verticalEdges_[i][j]) {
                    double x = j * cellWidth_;
                    double y = i * cellHeight_;
                    MatrixXd rect(4, 2); // 4x2 matrix for rectangle vertices
                    rect << x - wallThickness_ / 2, y,
                            x - wallThickness_ / 2, y + cellHeight_,
                            x + wallThickness_ / 2, y + cellHeight_,
                            x + wallThickness_ / 2, y;
                    obstacles.push_back(rect);
                }
            }
        }

        // Compute horizontal walls
        for (int i = 0; i < rows_; ++i) {
            for (int j = 0; j < cols_ - 1; ++j) {
                if (horizontalEdges_[i][j]) {
                    double x = j * cellWidth_;
                    double y = i * cellHeight_;
                    MatrixXd rect(4, 2); // 4x2 matrix for rectangle vertices
                    rect << x - wallThickness_ / 2, y - wallThickness_ / 2,
                            x - wallThickness_ / 2, y + wallThickness_ / 2,
                            x + wallThickness_ / 2 + cellWidth_, y + wallThickness_ / 2,
                            x + wallThickness_ / 2 + cellWidth_, y - wallThickness_ / 2;
                    obstacles.push_back(rect);
                }
            }
        }

        return obstacles;
    }

private:
    int rows_;
    int cols_;
    double cellWidth_;
    double cellHeight_;
    double wallThickness_;
    std::vector<std::vector<bool>> verticalEdges_;  // (rows x (cols - 1))
    std::vector<std::vector<bool>> horizontalEdges_; // ((rows - 1) x cols)
};