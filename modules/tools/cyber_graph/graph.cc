#include <iostream>

#include "cyber/common/log.h"
#include "cyber/init.h"
#include "cyber/service_discovery/topology_manager.h"

int main(int argc, char *argv[]) {
  std::cout << "Cyber Graph" << std::endl;
  apollo::cyber::Init(argv[0]);
  FLAGS_minloglevel = 3;
  FLAGS_alsologtostderr = 0;
  // FLAGS_colorlogtostderr = 0;

  // CyberTopologyMessage topologyMsg(val);

  apollo::cyber::proto::RoleAttributes attr;
  attr.set_host_name("");
  attr.set_process_id(0);

  auto channelManager =
      apollo::cyber::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  channelManager->AddChangeListener(
      [&attr](const apollo::cyber::proto::ChangeMsg &change_msg) {
        if (change_msg.change_type() ==
                apollo::cyber::proto::ChangeType::CHANGE_PARTICIPANT &&
            change_msg.operate_type() ==
                apollo::cyber::proto::OperateType::OPT_JOIN &&
            change_msg.role_type() ==
                apollo::cyber::proto::RoleType::ROLE_PARTICIPANT) {
          attr.CopyFrom(change_msg.role_attr());
        }
      });

  std::vector<apollo::cyber::proto::RoleAttributes> roleVec;
  channelManager->GetWriters(&roleVec);
  for (auto &role : roleVec) {
    std::cout << role.node_name() << std::endl;
    // topologyMsg.AddReaderWriter(role, true);
  }

  roleVec.clear();
  channelManager->GetReaders(&roleVec);
  for (auto &role : roleVec) {
    std::cout << role.node_name() << std::endl;
    // topologyMsg.AddReaderWriter(role, false);
  }

  //   Screen *s = Screen::Instance();

  //   signal(SIGWINCH, SigResizeHandle);
  //   signal(SIGINT, SigCtrlCHandle);

  //   s->SetCurrentRenderMessage(&topologyMsg);

  //   s->Init();
  //   s->Run();

  return 0;
}
