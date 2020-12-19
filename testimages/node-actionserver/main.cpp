
#include <actionlib/server/simple_action_server.h>

#include <shared_actions/DoDishesAction.h>

typedef actionlib::ActionServer<shared_actions::DoDishesAction> Server;

void onGoal(actionlib::ServerGoalHandle<shared_actions::DoDishesAction> h) {
    //shared_actions::DoDishesResult res;
    //res.total_dishes_cleaned = 15;
    //goal.getGoal().setSucceeded(res);
    //as->publishResult(goal, res);
}

void onCancel(actionlib::ServerGoalHandle<shared_actions::DoDishesAction> h) {
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_actionserver");
    ros::NodeHandle node;

    Server server(node, "test_action",
        &onGoal,
        onCancel,
        true);

    ros::spin();
    return 0;
}
