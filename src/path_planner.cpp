#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

#include <memory>

#include "utils.cpp"
#include "prototype_testing.cpp"

namespace path_planner
{

    // Temp Vairables
    std::vector<Cone> newDetections = {
        Cone(point{1.1, 2.2}, 5, false, 0.95, "orange"),
        Cone(point{3.1, 4.2}, 3, false, 0.90, "blue"),
        Cone(point{5.1, 6.2}, 2, true, 0.85, "yellow")};

    point car_pos = {0.0, 0.0};

    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("path_planner");

        // tag,x,y,direction,x_variance,y_variance,xy_covariance
        std::string trackFile = "../cone_csv/rectangle.csv";

        std::vector<Cone> cones = readTrackFile(trackFile);

        cones = compare_cones(newDetections, cones);
        std::vector<ConePath> tlVector = track_limit_derivation(cones);
        Path midline = midline_derivation(tlVector[0], tlVector[1]);
        Path raceline = raceline_derivation(tlVector[0], tlVector[1], midline);
    }

    // Function to compare new con positions to historical cone positions
    std::vector<Cone> compare_cones(std::vector<Cone> new_cones, std::vector<Cone> confirmed_cones)
    {
        for (int i = 0; i < new_cones.size(); i++)
        {
            double closest_distance = 1000000;
            int closest_cone = 0;
            for (int j = 0; j < confirmed_cones.size(); j++)
            {
                double distance = distanceBetweenCones(new_cones[i], confirmed_cones[j]);
                if (distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_cone = j;
                }
            }

            if (closest_distance < margin_of_error)
            {
                // Update cone position
                confirmed_cones[closest_cone].weightedConePosUpdate(new_cones[i].getPos());
            }
            else
            {
                // Add new cone to list
                confirmed_cones.push_back(new_cones[i]);
            }
        }

        return confirmed_cones;

    }
    
    // a function to derive the left and right track limits from the confirmed cones
    std::vector<ConePath> track_limit_derivation(std::vector<Cone> confirmed_cones)
    {
        ConeBST leftTrackLimit;
        ConeBST rightTrackLimit;

        leftTrackLimit.insert(closestLeftCone(confirmed_cones, car_pos));
        rightTrackLimit.insert(closestRightCone(confirmed_cones, car_pos));

        while (true)
        {
            // get the max element in the left track limit BST
            Cone leftMax = leftTrackLimit.findMax();
            std::vector<Cone> leftVectorNextCandidates = leftMax.scanArea(confirmed_cones, pred_future_cone_range, pred_future_cone_arc);
            if (leftVectorNextCandidates.size() == 1)
            {
                leftTrackLimit.insert(leftVectorNextCandidates[0]);
            }
            else if (leftVectorNextCandidates.size() > 1)
            {
                double maxScore = 0;
                int maxIndex = 0;
                for (int i = 0; i < leftVectorNextCandidates.size(); i++)
                {
                    double score = evaluateConeScore(leftVectorNextCandidates[i],leftMax);
                    if (score > maxScore)
                    {
                        maxScore = score;
                        maxIndex = i;
                    }
                }
                leftTrackLimit.insert(leftVectorNextCandidates[maxIndex]);
            }

            // get the max element in the right track limit BST
            Cone rightMax = rightTrackLimit.findMax();
            std::vector<Cone> rightVectorNextCandidates = rightMax.scanArea(confirmed_cones, pred_future_cone_range, pred_future_cone_arc);
            if (rightVectorNextCandidates.size() == 1)
            {
                rightTrackLimit.insert(rightVectorNextCandidates[0]);
            }
            else if (rightVectorNextCandidates.size() > 1)
            {
                double maxScore = 0;
                int maxIndex = 0;
                for (int i = 0; i < rightVectorNextCandidates.size(); i++)
                {
                    double score = evaluateConeScore(rightVectorNextCandidates[i],rightMax);
                    if (score > maxScore)
                    {
                        maxScore = score;
                        maxIndex = i;
                    }
                }
                rightTrackLimit.insert(rightVectorNextCandidates[maxIndex]);
            }
        }

        return {leftTrackLimit.outputToPath(), rightTrackLimit.outputToPath()};
    }

    // a function that from the left and the right track limits, derive the midline of the track
    Path midline_derivation(ConePath tlLeft, ConePath tlRight)
    {

        Path midline = Path(midpoint(tlLeft.getCone(0).getPos(), tlRight.getCone(0).getPos()));

        tlLeft.removeCone(tlLeft.getCone(0));
        tlRight.removeCone(tlRight.getCone(0));

        bool checkSide = false; // false for left, true for right

        while(true)
        {
            if (checkSide)
            {
                Cone rCone = closestOppositeCone(tlLeft.getCone(0), tlRight);
                midline.addPoint(midpoint(tlLeft.getCone(0).getPos(), rCone.getPos()));
                tlLeft.removeCone(tlLeft.getCone(0));
            }
            else
            {
                Cone lCone = closestOppositeCone(tlRight.getCone(0), tlLeft);
                midline.addPoint(midpoint(lCone.getPos(), tlRight.getCone(0).getPos()));
                tlRight.removeCone(tlRight.getCone(0));
            }
            if (tlLeft.size() == 0 || tlRight.size() == 0)
            {
                break;
            }
        }

        midline.loop_closure();

        return midline;
    }

    // a function that from the left and the right track limits and the midline of the track, derive the raceline
    Path raceline_derivation(ConePath tlLeft, ConePath tlRight, Path midline)
    {
        return midline;
    }
}