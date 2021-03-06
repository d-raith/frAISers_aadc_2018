// This autogenerated skeleton file illustrates how to build a server.
// You should copy it to another filename to avoid overwriting it.

#include "ControlComm.h"
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;

using namespace  ::control_thrift;

class ControlCommHandler : virtual public ControlCommIf {
 public:
  ControlCommHandler() {
    // Your initialization goes here
  }

  void ping() {
    // Your implementation goes here
    printf("ping\n");
  }

  CarStatus::type sendCommand(const ControlMessage& command) {
    // Your implementation goes here
    printf("sendCommand\n");
  }

  CarStatus::type getCarStatus() {
    // Your implementation goes here
    printf("getCarStatus\n");
  }

  double getRouteCosts(const ControlMessage& command) {
    // Your implementation goes here
    printf("getRouteCosts\n");
  }

};

int main(int argc, char **argv) {
  int port = 9090;
  ::apache::thrift::stdcxx::shared_ptr<ControlCommHandler> handler(new ControlCommHandler());
  ::apache::thrift::stdcxx::shared_ptr<TProcessor> processor(new ControlCommProcessor(handler));
  ::apache::thrift::stdcxx::shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
  ::apache::thrift::stdcxx::shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());
  ::apache::thrift::stdcxx::shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());

  TSimpleServer server(processor, serverTransport, transportFactory, protocolFactory);
  server.serve();
  return 0;
}

