/**
 * Autogenerated by Thrift Compiler (0.11.0)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 *  @generated
 */
#ifndef NNComm_H
#define NNComm_H

#include <thrift/TDispatchProcessor.h>
#include <thrift/async/TConcurrentClientSyncInfo.h>
#include "nn_thrift_types.h"

namespace nn_thrift {

#ifdef _MSC_VER
  #pragma warning( push )
  #pragma warning (disable : 4250 ) //inheriting methods via dominance 
#endif

class NNCommIf {
 public:
  virtual ~NNCommIf() {}
  virtual bool ping() = 0;
  virtual void getNNPrediction(NNImage& _return, const NNImage& raw_image) = 0;
};

class NNCommIfFactory {
 public:
  typedef NNCommIf Handler;

  virtual ~NNCommIfFactory() {}

  virtual NNCommIf* getHandler(const ::apache::thrift::TConnectionInfo& connInfo) = 0;
  virtual void releaseHandler(NNCommIf* /* handler */) = 0;
};

class NNCommIfSingletonFactory : virtual public NNCommIfFactory {
 public:
  NNCommIfSingletonFactory(const ::apache::thrift::stdcxx::shared_ptr<NNCommIf>& iface) : iface_(iface) {}
  virtual ~NNCommIfSingletonFactory() {}

  virtual NNCommIf* getHandler(const ::apache::thrift::TConnectionInfo&) {
    return iface_.get();
  }
  virtual void releaseHandler(NNCommIf* /* handler */) {}

 protected:
  ::apache::thrift::stdcxx::shared_ptr<NNCommIf> iface_;
};

class NNCommNull : virtual public NNCommIf {
 public:
  virtual ~NNCommNull() {}
  bool ping() {
    bool _return = false;
    return _return;
  }
  void getNNPrediction(NNImage& /* _return */, const NNImage& /* raw_image */) {
    return;
  }
};


class NNComm_ping_args {
 public:

  NNComm_ping_args(const NNComm_ping_args&);
  NNComm_ping_args& operator=(const NNComm_ping_args&);
  NNComm_ping_args() {
  }

  virtual ~NNComm_ping_args() throw();

  bool operator == (const NNComm_ping_args & /* rhs */) const
  {
    return true;
  }
  bool operator != (const NNComm_ping_args &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const NNComm_ping_args & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};


class NNComm_ping_pargs {
 public:


  virtual ~NNComm_ping_pargs() throw();

  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

typedef struct _NNComm_ping_result__isset {
  _NNComm_ping_result__isset() : success(false) {}
  bool success :1;
} _NNComm_ping_result__isset;

class NNComm_ping_result {
 public:

  NNComm_ping_result(const NNComm_ping_result&);
  NNComm_ping_result& operator=(const NNComm_ping_result&);
  NNComm_ping_result() : success(0) {
  }

  virtual ~NNComm_ping_result() throw();
  bool success;

  _NNComm_ping_result__isset __isset;

  void __set_success(const bool val);

  bool operator == (const NNComm_ping_result & rhs) const
  {
    if (!(success == rhs.success))
      return false;
    return true;
  }
  bool operator != (const NNComm_ping_result &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const NNComm_ping_result & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

typedef struct _NNComm_ping_presult__isset {
  _NNComm_ping_presult__isset() : success(false) {}
  bool success :1;
} _NNComm_ping_presult__isset;

class NNComm_ping_presult {
 public:


  virtual ~NNComm_ping_presult() throw();
  bool* success;

  _NNComm_ping_presult__isset __isset;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);

};

typedef struct _NNComm_getNNPrediction_args__isset {
  _NNComm_getNNPrediction_args__isset() : raw_image(false) {}
  bool raw_image :1;
} _NNComm_getNNPrediction_args__isset;

class NNComm_getNNPrediction_args {
 public:

  NNComm_getNNPrediction_args(const NNComm_getNNPrediction_args&);
  NNComm_getNNPrediction_args& operator=(const NNComm_getNNPrediction_args&);
  NNComm_getNNPrediction_args() {
  }

  virtual ~NNComm_getNNPrediction_args() throw();
  NNImage raw_image;

  _NNComm_getNNPrediction_args__isset __isset;

  void __set_raw_image(const NNImage& val);

  bool operator == (const NNComm_getNNPrediction_args & rhs) const
  {
    if (!(raw_image == rhs.raw_image))
      return false;
    return true;
  }
  bool operator != (const NNComm_getNNPrediction_args &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const NNComm_getNNPrediction_args & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};


class NNComm_getNNPrediction_pargs {
 public:


  virtual ~NNComm_getNNPrediction_pargs() throw();
  const NNImage* raw_image;

  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

typedef struct _NNComm_getNNPrediction_result__isset {
  _NNComm_getNNPrediction_result__isset() : success(false) {}
  bool success :1;
} _NNComm_getNNPrediction_result__isset;

class NNComm_getNNPrediction_result {
 public:

  NNComm_getNNPrediction_result(const NNComm_getNNPrediction_result&);
  NNComm_getNNPrediction_result& operator=(const NNComm_getNNPrediction_result&);
  NNComm_getNNPrediction_result() {
  }

  virtual ~NNComm_getNNPrediction_result() throw();
  NNImage success;

  _NNComm_getNNPrediction_result__isset __isset;

  void __set_success(const NNImage& val);

  bool operator == (const NNComm_getNNPrediction_result & rhs) const
  {
    if (!(success == rhs.success))
      return false;
    return true;
  }
  bool operator != (const NNComm_getNNPrediction_result &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const NNComm_getNNPrediction_result & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

typedef struct _NNComm_getNNPrediction_presult__isset {
  _NNComm_getNNPrediction_presult__isset() : success(false) {}
  bool success :1;
} _NNComm_getNNPrediction_presult__isset;

class NNComm_getNNPrediction_presult {
 public:


  virtual ~NNComm_getNNPrediction_presult() throw();
  NNImage* success;

  _NNComm_getNNPrediction_presult__isset __isset;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);

};

class NNCommClient : virtual public NNCommIf {
 public:
  NNCommClient(apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> prot) {
    setProtocol(prot);
  }
  NNCommClient(apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> iprot, apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> oprot) {
    setProtocol(iprot,oprot);
  }
 private:
  void setProtocol(apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> prot) {
  setProtocol(prot,prot);
  }
  void setProtocol(apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> iprot, apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> oprot) {
    piprot_=iprot;
    poprot_=oprot;
    iprot_ = iprot.get();
    oprot_ = oprot.get();
  }
 public:
  apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> getInputProtocol() {
    return piprot_;
  }
  apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> getOutputProtocol() {
    return poprot_;
  }
  bool ping();
  void send_ping();
  bool recv_ping();
  void getNNPrediction(NNImage& _return, const NNImage& raw_image);
  void send_getNNPrediction(const NNImage& raw_image);
  void recv_getNNPrediction(NNImage& _return);
 protected:
  apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> piprot_;
  apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> poprot_;
  ::apache::thrift::protocol::TProtocol* iprot_;
  ::apache::thrift::protocol::TProtocol* oprot_;
};

class NNCommProcessor : public ::apache::thrift::TDispatchProcessor {
 protected:
  ::apache::thrift::stdcxx::shared_ptr<NNCommIf> iface_;
  virtual bool dispatchCall(::apache::thrift::protocol::TProtocol* iprot, ::apache::thrift::protocol::TProtocol* oprot, const std::string& fname, int32_t seqid, void* callContext);
 private:
  typedef  void (NNCommProcessor::*ProcessFunction)(int32_t, ::apache::thrift::protocol::TProtocol*, ::apache::thrift::protocol::TProtocol*, void*);
  typedef std::map<std::string, ProcessFunction> ProcessMap;
  ProcessMap processMap_;
  void process_ping(int32_t seqid, ::apache::thrift::protocol::TProtocol* iprot, ::apache::thrift::protocol::TProtocol* oprot, void* callContext);
  void process_getNNPrediction(int32_t seqid, ::apache::thrift::protocol::TProtocol* iprot, ::apache::thrift::protocol::TProtocol* oprot, void* callContext);
 public:
  NNCommProcessor(::apache::thrift::stdcxx::shared_ptr<NNCommIf> iface) :
    iface_(iface) {
    processMap_["ping"] = &NNCommProcessor::process_ping;
    processMap_["getNNPrediction"] = &NNCommProcessor::process_getNNPrediction;
  }

  virtual ~NNCommProcessor() {}
};

class NNCommProcessorFactory : public ::apache::thrift::TProcessorFactory {
 public:
  NNCommProcessorFactory(const ::apache::thrift::stdcxx::shared_ptr< NNCommIfFactory >& handlerFactory) :
      handlerFactory_(handlerFactory) {}

  ::apache::thrift::stdcxx::shared_ptr< ::apache::thrift::TProcessor > getProcessor(const ::apache::thrift::TConnectionInfo& connInfo);

 protected:
  ::apache::thrift::stdcxx::shared_ptr< NNCommIfFactory > handlerFactory_;
};

class NNCommMultiface : virtual public NNCommIf {
 public:
  NNCommMultiface(std::vector<apache::thrift::stdcxx::shared_ptr<NNCommIf> >& ifaces) : ifaces_(ifaces) {
  }
  virtual ~NNCommMultiface() {}
 protected:
  std::vector<apache::thrift::stdcxx::shared_ptr<NNCommIf> > ifaces_;
  NNCommMultiface() {}
  void add(::apache::thrift::stdcxx::shared_ptr<NNCommIf> iface) {
    ifaces_.push_back(iface);
  }
 public:
  bool ping() {
    size_t sz = ifaces_.size();
    size_t i = 0;
    for (; i < (sz - 1); ++i) {
      ifaces_[i]->ping();
    }
    return ifaces_[i]->ping();
  }

  void getNNPrediction(NNImage& _return, const NNImage& raw_image) {
    size_t sz = ifaces_.size();
    size_t i = 0;
    for (; i < (sz - 1); ++i) {
      ifaces_[i]->getNNPrediction(_return, raw_image);
    }
    ifaces_[i]->getNNPrediction(_return, raw_image);
    return;
  }

};

// The 'concurrent' client is a thread safe client that correctly handles
// out of order responses.  It is slower than the regular client, so should
// only be used when you need to share a connection among multiple threads
class NNCommConcurrentClient : virtual public NNCommIf {
 public:
  NNCommConcurrentClient(apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> prot) {
    setProtocol(prot);
  }
  NNCommConcurrentClient(apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> iprot, apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> oprot) {
    setProtocol(iprot,oprot);
  }
 private:
  void setProtocol(apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> prot) {
  setProtocol(prot,prot);
  }
  void setProtocol(apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> iprot, apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> oprot) {
    piprot_=iprot;
    poprot_=oprot;
    iprot_ = iprot.get();
    oprot_ = oprot.get();
  }
 public:
  apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> getInputProtocol() {
    return piprot_;
  }
  apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> getOutputProtocol() {
    return poprot_;
  }
  bool ping();
  int32_t send_ping();
  bool recv_ping(const int32_t seqid);
  void getNNPrediction(NNImage& _return, const NNImage& raw_image);
  int32_t send_getNNPrediction(const NNImage& raw_image);
  void recv_getNNPrediction(NNImage& _return, const int32_t seqid);
 protected:
  apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> piprot_;
  apache::thrift::stdcxx::shared_ptr< ::apache::thrift::protocol::TProtocol> poprot_;
  ::apache::thrift::protocol::TProtocol* iprot_;
  ::apache::thrift::protocol::TProtocol* oprot_;
  ::apache::thrift::async::TConcurrentClientSyncInfo sync_;
};

#ifdef _MSC_VER
  #pragma warning( pop )
#endif

} // namespace

#endif
