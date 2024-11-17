#include <vector>
#include <cmath>

#include <memory>


#include <iostream>

#include "utils.h"
#include "ConfigLoader.h"
#include "TrackLoader.h"
#include "GridPlotter.h"

#include "path.cpp"
#include "cone.cpp"
#include "conePath.cpp"
//#include "coneBST.cpp"


// Temp Vairables
// std::vector<Cone> newDetections = {
//     Cone(point{1.1, 2.2}, 5, false, 0.95, "orange"),
//     Cone(point{3.1, 4.2}, 3, false, 0.90, "blue"),
//     Cone(point{5.1, 6.2}, 2, true, 0.85, "yellow")};

ConfigLoader config(CONFIG_PATH);

point car_pos = {config.car_start_pos_x, config.car_start_pos_y};


// reload cone positions based on covariance
std::vector<Cone> reloadCones(std::vector<Cone> cones)
{
    for (int i = 0; i < cones.size(); i++)
    {
        //cones[i].reloadPos();
    }
    return cones;
}

// return the closest cone from a point
Cone closestCone(const std::vector<Cone> &cones, point p)
{
    double closest_distance = 1000000;
    int closest_cone = 0;
    for (int i = 0; i < cones.size(); i++)
    {
        double distance = cones[i].distanceToCone(p);
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
        double distance = cones[i].distanceToCone(car_pos);
        double angle = cones[i].angleBetweenPointAndCone(car_pos);
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
        double distance = cones[i].distanceToCone(car_pos);
        double angle = cones[i].angleBetweenPointAndCone(car_pos);
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
    double distance = cone.distanceBetweenCones(prev_cone);
    double angle = cone.angleBetweenCones(prev_cone);
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
        double distance = cone.distanceBetweenCones(oppositeTLVector.getCone(i));
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

// Function to compare new con positions to historical cone positions
std::vector<Cone> compare_cones(std::vector<Cone> new_cones, std::vector<Cone> confirmed_cones)
{
    for (int i = 0; i < new_cones.size(); i++)
    {
        double closest_distance = 1000000;
        int closest_cone = 0;
        for (int j = 0; j < confirmed_cones.size(); j++)
        {
            double distance = new_cones[i].distanceBetweenCones(confirmed_cones[j]);
            if (distance < closest_distance)
            {
                closest_distance = distance;
                closest_cone = j;
            }
        }

        if (closest_distance < config.margin_of_error)
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
    ConePath leftTrackLimit;
    ConePath rightTrackLimit;

    for (int i = 0; i < confirmed_cones.size(); i++)
    {
        if (confirmed_cones[i].getConeType() == "big_orange")
        {
            if (confirmed_cones[i].angleBetweenPointAndCone(car_pos) < 0)
            {
                leftTrackLimit.addCone(confirmed_cones[i]);
            }
            else
            {
                rightTrackLimit.addCone(confirmed_cones[i]);
            }
        }
    }

    if (leftTrackLimit.size() == 0)
    {
        std::cout << "No left Big Orange Found" << std::endl;
    }
    if (rightTrackLimit.size() == 0)
    {
        std::cout << "No right Big Orange Found" << std::endl;
    }
    
    int iterations = 0;
    while (iterations < config.max_wile_loop_iterations)
    {
        iterations++;
        Cone lastLeftCone = leftTrackLimit.getCone(leftTrackLimit.size() - 1);

        std::vector<Cone> leftVectorNextCandidates = lastLeftCone.scanArea(confirmed_cones, config.pred_future_cone_range, config.pred_future_cone_arc);

        if (leftVectorNextCandidates.size() == 1)
        {
            leftTrackLimit.addCone(leftVectorNextCandidates[0]);
        }
        else if (leftVectorNextCandidates.size() > 1)
        {
            double maxScore = 0;
            int maxIndex = 0;
            for (int i = 0; i < leftVectorNextCandidates.size(); i++)
            {
                double score = evaluateConeScore(leftVectorNextCandidates[i], lastLeftCone);
                if (score > maxScore)
                {
                    maxScore = score;
                    maxIndex = i;
                }
            }
            leftTrackLimit.addCone(leftVectorNextCandidates[maxIndex]);
        }
        if (leftTrackLimit.getCone(leftTrackLimit.size()-1).getConeType() == "big_orange")
        {
            break;
        }
    }

    iterations = 0;
    while (iterations < config.max_wile_loop_iterations)
    {
        iterations++;
        Cone lastRightCone = rightTrackLimit.getCone(rightTrackLimit.size() - 1);

        std::vector<Cone> rightVectorNextCandidates = lastRightCone.scanArea(confirmed_cones, config.pred_future_cone_range, config.pred_future_cone_arc);

        if (rightVectorNextCandidates.size() == 1)
        {
            rightTrackLimit.addCone(rightVectorNextCandidates[0]);
        }
        else if (rightVectorNextCandidates.size() > 1)
        {
            double maxScore = 0;
            int maxIndex = 0;
            for (int i = 0; i < rightVectorNextCandidates.size(); i++)
            {
                //if ()
                double score = evaluateConeScore(rightVectorNextCandidates[i], lastRightCone);
                if (score > maxScore)
                {
                    maxScore = score;
                    maxIndex = i;
                }
            }
            rightTrackLimit.addCone(rightVectorNextCandidates[maxIndex]);
        }
        if (rightTrackLimit.getCone(rightTrackLimit.size()-1).getConeType() == "big_orange")
        {
            break;
        }
    }

    return {leftTrackLimit, rightTrackLimit};

        // ConeBST leftTrackLimit;
        // ConeBST rightTrackLimit;

        // leftTrackLimit.insert(closestLeftCone(confirmed_cones, car_pos));
        // rightTrackLimit.insert(closestRightCone(confirmed_cones, car_pos));

        // while (true)
        // {
        //     // get the max element in the left track limit BST
        //     Cone leftMax = leftTrackLimit.findMax();
        //     std::vector<Cone> leftVectorNextCandidates = leftMax.scanArea(confirmed_cones, config.pred_future_cone_range, config.pred_future_cone_arc);
        //     if (leftVectorNextCandidates.size() == 1)
        //     {
        //         leftTrackLimit.insert(leftVectorNextCandidates[0]);
        //     }
        //     else if (leftVectorNextCandidates.size() > 1)
        //     {
        //         double maxScore = 0;
        //         int maxIndex = 0;
        //         for (int i = 0; i < leftVectorNextCandidates.size(); i++)
        //         {
        //             double score = evaluateConeScore(leftVectorNextCandidates[i],leftMax);
        //             if (score > maxScore)
        //             {
        //                 maxScore = score;
        //                 maxIndex = i;
        //             }
        //         }
        //         leftTrackLimit.insert(leftVectorNextCandidates[maxIndex]);
        //     }

        //     // get the max element in the right track limit BST
        //     Cone rightMax = rightTrackLimit.findMax();
        //     std::vector<Cone> rightVectorNextCandidates = rightMax.scanArea(confirmed_cones, config.pred_future_cone_range, config.pred_future_cone_arc);
        //     if (rightVectorNextCandidates.size() == 1)
        //     {
        //         rightTrackLimit.insert(rightVectorNextCandidates[0]);
        //     }
        //     else if (rightVectorNextCandidates.size() > 1)
        //     {
        //         double maxScore = 0;
        //         int maxIndex = 0;
        //         for (int i = 0; i < rightVectorNextCandidates.size(); i++)
        //         {
        //             double score = evaluateConeScore(rightVectorNextCandidates[i],rightMax);
        //             if (score > maxScore)
        //             {
        //                 maxScore = score;
        //                 maxIndex = i;
        //             }
        //         }
        //         rightTrackLimit.insert(rightVectorNextCandidates[maxIndex]);
        //     }
        // }

        // return {leftTrackLimit.outputToPath(), rightTrackLimit.outputToPath()};


}

// a function that from the left and the right track limits, derive the midline of the track
Path midline_derivation(ConePath tlLeft, ConePath tlRight)
{

    Path midline = Path(midpoint(tlLeft.getCone(0).getPos(), tlRight.getCone(0).getPos()));

    tlLeft.removeCone(tlLeft.getCone(0));
    tlRight.removeCone(tlRight.getCone(0));

    bool checkSide = false; // false for left, true for right

    int iterations = 0;
    while (iterations < config.max_wile_loop_iterations)
    {
        iterations++;
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

    midline.loop_closure(config.loop_closure_MOE);

    return midline;
}

// a function that from the left and the right track limits and the midline of the track, derive the raceline
Path raceline_derivation(ConePath tlLeft, ConePath tlRight, Path midline)
{
    return midline;
}

int main(int argc, char **argv)
{
    //std::cout << TRACK_PATH  + config.track<< std::endl;
    std::vector<Cone> cones = TrackLoader(TRACK_PATH + config.track).cones;

    //for (auto i : cones)
    //    std::cout << i.toString() << ' ';

    cones = compare_cones(std::vector<Cone>(), cones);
    //for (auto i : cones)
    //    std::cout << i.toString() << ' ';

    std::vector<ConePath> tlVector = track_limit_derivation(cones);
    for (int i=0; i < tlVector[0].size(); i++)
        std::cout << tlVector[0].getCone(i).toString() << ' ';

    for (int i = 0; i < tlVector[1].size(); i++)
        std::cout << tlVector[1].getCone(i).toString() << ' ';

    Path midline = midline_derivation(tlVector[0], tlVector[1]);
    // std::cout << midline << std::endl;

    // Path raceline = raceline_derivation(tlVector[0], tlVector[1], midline);
    //std::cout << raceline << std::endl;

    // Plot the grid
    GridPlotter plotter(800, 800, 10.0);
    plotter.plotGrid(cones, tlVector[0], tlVector[1], midline, "grid_plot.png");
}