#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <skywalker/ur5eAction.h>

class ur5eAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<skywalker::ur5eAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  skywalker::ur5eFeedback feedback_;
  skywalker::ur5eResult result_;

public:

  ur5eAction(std::string name) :
    as_(nh_, name, boost::bind(&ur5eAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~ur5eAction(void)
  {
  }

  void executeCB(const skywalker::ur5eGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.feedback.clear();
    feedback_.feedback.push_back(0);
    feedback_.feedback.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->goal, feedback_.feedback[0], feedback_.feedback[1]);

    // start executing the action
    for(int i=1; i<=goal->goal; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.feedback.push_back(feedback_.feedback[i] + feedback_.feedback[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.result = feedback_.feedback;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");

  ur5eAction action("action_server");
  ros::spin();

  return 0;
}
