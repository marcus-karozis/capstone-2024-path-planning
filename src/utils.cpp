#include <cmath>
#include <iostream>
#include <vector>

namespace path_planner
{

    // Config Variables
    double margin_of_error = 0.1;
    double cone_detection_MOE_rate = 0.1;

    // predicted future cone pos config
    double pred_future_cone_range = 3.0;
    double pred_future_cone_arc = 45.0;

    // loop closure margin of error config
    double loop_closure_MOE = 0.5;

    // point struct
    struct point
    {
        double x;
        double y;
        double x_variance = 0;
        double y_variance = 0;
        double xy_covariance = 0;
        double ground_truth_x = this->x;
        double ground_truth_y = this->y;
    };

    class Cone
    {
        private:
            point pos;
            int num_detections;
            bool discarded;
            double accuracy_confidence;
            std::string cone_type;

        public:

            Cone(point pos, int num_detections, bool discarded, double accuracy_confidence, std::string cone_type)
            {
                this->pos = pos;
                this->num_detections = num_detections;
                this->discarded = discarded;
                this->accuracy_confidence = accuracy_confidence;
                this->cone_type = cone_type;
            }
            Cone(std::string tag, double x, double y, double direction, double x_variance, double y_variance, double xy_covariance)
            {
                this->pos = point{x, y};
                this->num_detections = 1;
                this->discarded = false;
                this->accuracy_confidence = 1;
                this->cone_type = tag;
            }

            point getPos() const
            {
                return pos;
            }
            
            void weightedConePosUpdate(point new_pos)
            {
                pos.x = (pos.x * num_detections + new_pos.x) / (num_detections + 1);
                pos.y = (pos.y * num_detections + new_pos.y) / (num_detections + 1);
                num_detections++;
            }
            void evaluatePosValidity(Cone prev_cone)
            {
                // Check if cone falls within predicted future cone positions
                // based on track rules
                double distance = distanceBetweenCones(*this, prev_cone);
                double angle = angleBetweenCones(*this, prev_cone);
                
            }

            // a function to scans area in front of cone based on parameters to find subsequent cones
            std::vector<Cone> scanArea(std::vector<Cone> cones, double range, double arc)
            {
                std::vector<Cone> cones_in_range;
                for (int i = 0; i < cones.size(); i++)
                {
                    double distance = distanceBetweenCones(*this, cones[i]);
                    double angle = angleBetweenCones(*this, cones[i]);
                    if (distance < range && angle < arc)
                    {
                        cones_in_range.push_back(cones[i]);
                    }
                }
                return cones_in_range;
            }
    };

    // Cone BST Class
    class ConeBST
    {
        private:
            struct Node
            {
                Cone cone;
                Node *left;
                Node *right;
                bool finalNode=false;
            };

            Node *root;

            Node *makeEmpty(Node *node)
            {
                if (node == nullptr)
                {
                    return nullptr;
                }

                makeEmpty(node->left);
                makeEmpty(node->right);
                delete node;
                return nullptr;
            }

            Node *insert(Node *node, Cone cone)
            {
                if (node == nullptr)
                {
                    return new Node{cone, nullptr, nullptr};
                }

                if (cone.getPos().x < node->cone.getPos().x)
                {
                    node->left = insert(node->left, cone);
                }
                else
                {
                    node->right = insert(node->right, cone);
                }

                return node;
            }

            Node *findMin(Node *node)
            {
                if (node == nullptr)
                {
                    return nullptr;
                }

                if (node->left == nullptr)
                {
                    return node;
                }

                return findMin(node->left);
            }

            Node *findMax(Node *node)
            {
                if (node != nullptr)
                {
                    while (node->right != nullptr)
                    {
                        node = node->right;
                    }
                }

                return node;
            }

            Node *remove(Node *node, Cone cone)
            {
                Node *temp;
                if (node == nullptr)
                {
                    return nullptr;
                }
                else if (cone.getPos().x < node->cone.getPos().x)
                {
                    node->left = remove(node->left, cone);
                }
                else if (cone.getPos().x > node->cone.getPos().x)
                {
                    node->right = remove(node->right, cone);
                }
                else if (node->left && node->right)
                {
                    temp = findMin(node->right);
                    node->cone = temp->cone;
                    node->right = remove(node->right, node->cone);
                }
                else
                {
                    temp = node;
                    if (node->left == nullptr)
                    {
                        node = node->right;
                    }
                    else if (node->right == nullptr)
                    {
                        node = node->left;
                    }
                    delete temp;
                }

                return temp;
            }

            void inOrder(Node *node)
            {
                if (node == nullptr)
                {
                    return;
                }

                inOrder(node->left);
                std::cout << node->cone.getPos().x << " ";
                inOrder(node->right);
            }

            void toPath(Node *node, std::vector<Cone> &cones)
            {
                if (node == nullptr)
                {
                    return;
                }

                toPath(node->left, cones);
                cones.push_back(node->cone);
                toPath(node->right, cones);
            }

            Node *find(Node *node, double x) // NEED TO REWRITE THIS FUNCTION
            {
                if (node == nullptr)
                {
                    return nullptr;
                }
                else if (x < node->cone.getPos().x)
                {
                    return find(node->left, x);
                }
                else if (x > node->cone.getPos().x)
                {
                    return find(node->right, x);
                }
                else
                {
                    return node;
                }
            }

        public:
            ConeBST()
            {
                root = nullptr;
            }

            ~ConeBST()
            {
                root = makeEmpty(root);
            }

            void insert(Cone cone)
            {
                root = insert(root, cone);
            }

            void remove(Cone cone)
            {
                root = remove(root, cone);
            }

            void display()
            {
                inOrder(root);
                std::cout << std::endl;
            }

            ConePath outputToPath()
            {
                std::vector<Cone> cones;
                toPath(root, cones);
                return ConePath(cones);
            }

            Cone findMin()
            {
                return findMin(root)->cone;
            }

            Cone findMax()
            {
                return findMax(root)->cone;
            }

            Cone find(double x)
            {
                return find(root, x)->cone;
            }
    };

    // A Class for storing a path of points to be used by the midline vector, and raceline vector
    class Path
    {
        private:
            std::vector<point> path;

        public:
            Path()
            {
                path = std::vector<point>();
            }
            Path(std::vector<point> path)
            {
                this->path = path;
            }
            Path(point p)
            {
                path = std::vector<point>();
                path.push_back(p);
            }

            void addPoint(point p)
            {
                path.push_back(p);
            }

            void removePoint(point p)
            {
                for (int i = 0; i < path.size(); i++)
                {
                    if (path[i].x == p.x && path[i].y == p.y)
                    {
                        path.erase(path.begin() + i);
                        break;
                    }
                }
            }

            void display()
            {
                for (int i = 0; i < path.size(); i++)
                {
                    std::cout << "(" << path[i].x << ", " << path[i].y << ") ";
                }
                std::cout << std::endl;
            }

            // checks to see if the last point in the path is close to the first point in the path based on Closure MOE
            void loop_closure()
            {
                if (distanceBetweenPoints(path[0], path[path.size() - 1]) < loop_closure_MOE)
                {
                    removePoint(path.back());
                    path.push_back(path[0]);
                }
            }
    };

    // A Class for storing a path of cones to be used by the track limit vectors
    class ConePath
    {
        private:
            std::vector<Cone> path;

        public:
            ConePath()
            {
                path = std::vector<Cone>();
            }
            ConePath(std::vector<Cone> path)
            {
                this->path = path;
            }
            ConePath(Cone c)
            {
                path = std::vector<Cone>();
                path.push_back(c);
            }

            void addCone(Cone c)
            {
                path.push_back(c);
            }

            void removeCone(Cone c)
            {
                for (int i = 0; i < path.size(); i++)
                {
                    if (path[i].getPos().x == c.getPos().x && path[i].getPos().y == c.getPos().y)
                    {
                        path.erase(path.begin() + i);
                        break;
                    }
                }
            }

            void display()
            {
                for (int i = 0; i < path.size(); i++)
                {
                    std::cout << "(" << path[i].getPos().x << ", " << path[i].getPos().y << ") ";
                }
                std::cout << std::endl;
            }

            int coneIndex(Cone c)
            {
                for (int i = 0; i < path.size(); i++)
                {
                    if (path[i].getPos().x == c.getPos().x && path[i].getPos().y == c.getPos().y)
                    {
                        return i;
                    }
                }
                return -1;
            }

            Cone getCone(int index)
            {
                return path[index];
            }

            int size()
            {
                return path.size();
            }

            // checks to see if the last point in the path is close to the first point in the path based on Closure MOE
            void loop_closure()
            {
                if (distanceBetweenCones(path[0], path[path.size() - 1]) < loop_closure_MOE)
                {
                    removeCone(path.back());
                    path.push_back(path[0]);
                }
            }
    };


    // return the distance between two cones as a double
    double distanceBetweenCones(const Cone &cone1, const Cone &cone2)
    {
        double dx = cone2.getPos().x - cone1.getPos().x;
        double dy = cone2.getPos().y - cone1.getPos().y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // return the distance between two points as a double
    double distanceBetweenPoints(point p1, point p2)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // return the angle between two cones in degrees
    double angleBetweenCones(const Cone &cone1, const Cone &cone2)
    {
        double dx = cone2.getPos().x - cone1.getPos().x;
        double dy = cone2.getPos().y - cone1.getPos().y;
        return std::atan2(dy, dx) * 180 / M_PI;
    }

    // return the angle between a point and a cone in degrees
    double angleBetweenPointAndCone(const Cone &cone, point p)
    {
        double dx = cone.getPos().x - p.x;
        double dy = cone.getPos().y - p.y;
        return std::atan2(dy, dx) * 180 / M_PI;
    }

    // return the distance between a point and a cone
    double distanceToCone(const Cone &cone, point p)
    {
        double dx = cone.getPos().x - p.x;
        double dy = cone.getPos().y - p.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // return the closest cone from a point
    Cone closestCone(const std::vector<Cone> &cones, point p)
    {
        double closest_distance = 1000000;
        int closest_cone = 0;
        for (int i = 0; i < cones.size(); i++)
        {
            double distance = distanceToCone(cones[i], p);
            if (distance < closest_distance)
            {
                closest_distance = distance;
                closest_cone = i;
            }
        }
        return cones[closest_cone];
    }

    // return the closest cone on the left of the car
    Cone closestLeftCone(const std::vector<Cone> &cones, point car_pos)
    {
        double closest_distance = 1000000;
        int closest_cone = 0;
        for (int i = 0; i < cones.size(); i++)
        {
            double distance = distanceToCone(cones[i], car_pos);
            double angle = angleBetweenPointAndCone(cones[i], car_pos);
            if (distance < closest_distance && angle < 0)
            {
                closest_distance = distance;
                closest_cone = i;
            }
        }
        return cones[closest_cone];
    }

    // return the closest cone on the right of the car
    Cone closestRightCone(const std::vector<Cone> &cones, point car_pos)
    {
        double closest_distance = 1000000;
        int closest_cone = 0;
        for (int i = 0; i < cones.size(); i++)
        {
            double distance = distanceToCone(cones[i], car_pos);
            double angle = angleBetweenPointAndCone(cones[i], car_pos);
            if (distance < closest_distance && angle > 0)
            {
                closest_distance = distance;
                closest_cone = i;
            }
        }
        return cones[closest_cone];
    }

    // a function that evaluates a score for the likeliness and accuracy of a cone being the next cone in the track limit vector based on accuracy score, straightness, and distance
    double evaluateConeScore(Cone cone, Cone prev_cone)
    {
        double distance = distanceBetweenCones(cone, prev_cone);
        double angle = angleBetweenCones(cone, prev_cone);
        double score = 0;
        if (distance < 1)
        {
            score += 0.5;
        }
        if (angle < 10)
        {
            score += 0.3;
        }
        if (cone.getPos().x > prev_cone.getPos().x)
        {
            score += 0.2;
        }
        return score;
    }

    // a function that finds the closest cone from the opposite side TL Vector
    Cone closestOppositeCone(Cone cone, ConePath oppositeTLVector)
    {
        double closest_distance = 1000000;
        int closest_cone = 0;
        for (int i = 0; i < oppositeTLVector.size(); i++)
        {
            double distance = distanceBetweenCones(cone, oppositeTLVector.getCone(i));
            if (distance < closest_distance)
            {
                closest_distance = distance;
                closest_cone = i;
            }
        }
        return oppositeTLVector.getCone(closest_cone);
    }

    // a function that finds the midpont of two points
    point midpoint(point p1, point p2)
    {
        return point{(p1.x + p2.x) / 2, (p1.y + p2.y) / 2};
    }

}