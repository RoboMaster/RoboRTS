/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_SDK_DISPATCH_H
#define ROBORTS_SDK_DISPATCH_H
#include <iostream>       // std::cout
#include <functional>     // std::ref
#include <thread>         // std::thread
#include <future>         // std::promise, std::future
#include <map>
#include <tuple>

#include "handle.h"

namespace roborts_sdk {
class Handle;

template<typename Cmd>
class SubscriptionCallback {
 public:
  using SharedMessage = typename std::shared_ptr<Cmd>;
  using CallbackType = typename std::function<void(const SharedMessage)>;

  SubscriptionCallback(CallbackType function) :
      callback_function_(function) {

  }
  ~SubscriptionCallback() = default;
  void Dispatch(std::shared_ptr<MessageHeader> message_header,
                SharedMessage message) {
    callback_function_(message);
  };
 private:
  CallbackType callback_function_;
};

template<typename Cmd, typename Ack>
class ServiceCallback {
 public:
  using SharedRequest = typename std::shared_ptr<Cmd>;
  using SharedResponse = typename std::shared_ptr<Ack>;
  using CallbackType = typename std::function<void(const SharedRequest, SharedResponse)>;
  ServiceCallback(CallbackType function) :
      callback_function_(function) {

  }
  ~ServiceCallback() = default;
  void Dispatch(std::shared_ptr<MessageHeader> message_header,
                SharedRequest request,
                SharedResponse response) {
    callback_function_(request, response);
  };
 private:
  CallbackType callback_function_;
};

class SubscriptionBase {
 public:
  SubscriptionBase(std::shared_ptr<Handle> handle,
                   uint8_t cmd_set, uint8_t cmd_id,
                   uint8_t sender, uint8_t receiver) :
      handle_(handle) {
    cmd_info_ = std::make_shared<CommandInfo>();
    cmd_info_->cmd_id = cmd_id;
    cmd_info_->cmd_set = cmd_set;
    cmd_info_->receiver = receiver;
    cmd_info_->sender = sender;
    cmd_info_->need_ack = false;
  }
  ~SubscriptionBase() = default;
  std::shared_ptr<Handle> GetHandle() {
    return handle_;
  }
  std::shared_ptr<CommandInfo> GetCommandInfo() {
    return cmd_info_;
  }
  std::shared_ptr<MessageHeader> CreateMessageHeader() {
    return std::shared_ptr<MessageHeader>(new MessageHeader);
  }

  virtual std::shared_ptr<void> CreateMessage() = 0;
  virtual void HandleMessage(std::shared_ptr<MessageHeader> message_header, std::shared_ptr<void> message) = 0;
 protected:
  std::shared_ptr<Handle> handle_;
  std::shared_ptr<CommandInfo> cmd_info_;

};

template<typename Cmd>
class Subscription : public SubscriptionBase {
 public:
  using SharedMessage = typename std::shared_ptr<Cmd>;
  using CallbackType = typename std::function<void(const SharedMessage)>;

  Subscription(std::shared_ptr<Handle> handle,
               uint8_t cmd_set, uint8_t cmd_id,
               uint8_t sender, uint8_t receiver,
               CallbackType &&function) :
      SubscriptionBase(handle, cmd_set, cmd_id, sender, receiver),
      callback_(std::forward<CallbackType>(function)) {
    cmd_info_->length = sizeof(Cmd);
  }
  ~Subscription() = default;
  std::shared_ptr<void> CreateMessage() {
    return std::shared_ptr<void>(new Cmd);
  }
  void HandleMessage(std::shared_ptr<MessageHeader> message_header, std::shared_ptr<void> message) {
    auto typed_message = std::static_pointer_cast<Cmd>(message);
    callback_.Dispatch(message_header, typed_message);
  }
 private:
  SubscriptionCallback<Cmd> callback_;
};

class PublisherBase {
 public:
  PublisherBase(std::shared_ptr<Handle> handle,
                uint8_t cmd_set, uint8_t cmd_id,
                uint8_t sender, uint8_t receiver) :
      handle_(handle) {
    cmd_info_ = std::make_shared<CommandInfo>();
    cmd_info_->cmd_id = cmd_id;
    cmd_info_->cmd_set = cmd_set;
    cmd_info_->receiver = receiver;
    cmd_info_->sender = sender;
    cmd_info_->need_ack = false;
  }
  ~PublisherBase() = default;
  std::shared_ptr<Handle> GetHandle() {
    return handle_;
  }
  std::shared_ptr<CommandInfo> GetCommandInfo() {
    return cmd_info_;
  }

 protected:
  std::shared_ptr<Handle> handle_;
  std::shared_ptr<CommandInfo> cmd_info_;
};

template<typename Cmd>
class Publisher : public PublisherBase {
 public:
  Publisher(std::shared_ptr<Handle> handle,
            uint8_t cmd_set, uint8_t cmd_id,
            uint8_t sender, uint8_t receiver) :
      PublisherBase(handle, cmd_set, cmd_id, sender, receiver) {
    cmd_info_->length = sizeof(Cmd);
  }
  ~Publisher() = default;

  void Publish(Cmd &message) {
    bool ret = GetHandle()->GetProtocol()->SendMessage(GetCommandInfo().get(), &message);
    if (!ret) {
      DLOG_ERROR << "send message failed!";
    }
  }
};

class ClientBase {
 public:
  ClientBase(std::shared_ptr<Handle> handle,
             uint8_t cmd_set, uint8_t cmd_id,
             uint8_t sender, uint8_t receiver) :
      handle_(handle) {
    cmd_info_ = std::make_shared<CommandInfo>();
    cmd_info_->cmd_id = cmd_id;
    cmd_info_->cmd_set = cmd_set;
    cmd_info_->receiver = receiver;
    cmd_info_->sender = sender;
    cmd_info_->need_ack = true;
  }
  ~ClientBase() = default;
  std::shared_ptr<Handle> GetHandle() {
    return handle_;
  }
  std::shared_ptr<CommandInfo> GetCommandInfo() {
    return cmd_info_;
  }
  std::shared_ptr<MessageHeader> CreateRequestHeader() {
    return std::shared_ptr<MessageHeader>(new MessageHeader);
  }

  virtual std::shared_ptr<void> CreateResponse() = 0;
  virtual void HandleResponse(std::shared_ptr<MessageHeader> request_header,
                              std::shared_ptr<void> response) = 0;

 protected:
  std::shared_ptr<Handle> handle_;
  std::shared_ptr<CommandInfo> cmd_info_;

};

template<typename Cmd, typename Ack>
class Client : public ClientBase {
 public:
  using SharedRequest  = typename std::shared_ptr<Cmd>;
  using SharedResponse = typename std::shared_ptr<Ack>;

  using Promise        = std::promise<SharedResponse>;
  using SharedPromise  = std::shared_ptr<Promise>;
  using SharedFuture   = std::shared_future<SharedResponse>;
  using CallbackType   = std::function<void(SharedFuture)>;

  Client(std::shared_ptr<Handle> handle,
         uint8_t cmd_set, uint8_t cmd_id,
         uint8_t sender, uint8_t receiver) :
      ClientBase(handle, cmd_set, cmd_id, sender, receiver) {
    cmd_info_->length = sizeof(Ack);
  }
  ~Client() = default;

  std::shared_ptr<void> CreateResponse() {
    return std::shared_ptr<void>(new Ack());
  }

  void HandleResponse(std::shared_ptr<MessageHeader> request_header, std::shared_ptr<void> response) {

    std::unique_lock<std::mutex> lock(pending_requests_mutex_);
    auto typed_response = std::static_pointer_cast<Ack>(response);
    //TODO: Determine key: seq_num or session_id
    uint8_t session_id = request_header->session_id;

    if(pending_requests_.count(session_id) == 0){
      LOG_ERROR<<"Received invalid session_id. Ignoring...";
      return;
    }
    auto call_promise = std::get<0>(pending_requests_[session_id]);
    auto callback = std::get<1>(pending_requests_[session_id]);
    auto future = std::get<2>(pending_requests_[session_id]);
    pending_requests_.erase(session_id);
    // Unlock here to allow the service to be called recursively from one of its callbacks.
    lock.unlock();

    call_promise->set_value(typed_response);
    callback(future);
  }

  SharedFuture
  AsyncSendRequest(SharedRequest request) {
    return AsyncSendRequest(request, [](SharedFuture) {});
  }

  template<typename CallbackType>
  SharedFuture AsyncSendRequest(SharedRequest request, CallbackType &&cb) {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    auto request_header = std::make_shared<MessageHeader>();
    bool ret = GetHandle()->GetProtocol()->SendRequest(GetCommandInfo().get(), request_header.get(), request.get());

    if (!ret) {
      LOG_ERROR << "Async_send_request failed!";
    }

    SharedPromise call_promise = std::make_shared<Promise>();
    SharedFuture f(call_promise->get_future());
    //TODO: Determine key: seq_num or session_id
    pending_requests_[request_header->session_id] = std::make_tuple(call_promise, std::forward<CallbackType>(cb), f);

    return f;
  }

 private:
  //TODO: Determine key: seq_num or session_id
  std::map<uint8_t, std::tuple<SharedPromise, CallbackType, SharedFuture>> pending_requests_;
  std::mutex pending_requests_mutex_;
};

class ServiceBase {
 public:
  ServiceBase(std::shared_ptr<Handle> handle,
              uint8_t cmd_set, uint8_t cmd_id,
              uint8_t sender, uint8_t receiver) :
      handle_(handle) {
    cmd_info_ = std::make_shared<CommandInfo>();
    cmd_info_->cmd_id = cmd_id;
    cmd_info_->cmd_set = cmd_set;
    cmd_info_->receiver = receiver;
    cmd_info_->sender = sender;
    cmd_info_->need_ack = true;
  }
  ~ServiceBase() = default;

  std::shared_ptr<Handle> GetHandle() {
    return handle_;
  }
  std::shared_ptr<CommandInfo> GetCommandInfo() {
    return cmd_info_;
  }
  std::shared_ptr<MessageHeader> CreateRequestHeader() {
    return std::shared_ptr<MessageHeader>(new MessageHeader);
  }

  virtual std::shared_ptr<void> CreateRequest() = 0;
  virtual void HandleRequest(
      std::shared_ptr<MessageHeader> request_header,
      std::shared_ptr<void> request) = 0;
 protected:
  std::shared_ptr<Handle> handle_;
  std::shared_ptr<CommandInfo> cmd_info_;
};

template<typename Cmd, typename Ack>
class Service : public ServiceBase {

 public:
  using SharedRequest = typename std::shared_ptr<Cmd>;
  using SharedResponse = typename std::shared_ptr<Ack>;
  using CallbackType = typename std::function<void(const SharedRequest, SharedResponse)>;

  Service(std::shared_ptr<Handle> handle,
          uint8_t cmd_set, uint8_t cmd_id,
          uint8_t sender, uint8_t receiver,
          CallbackType &&function) :
      ServiceBase(handle, cmd_set, cmd_id, sender, receiver),
      callback_(std::forward<CallbackType>(function)) {
    cmd_info_->length = sizeof(Cmd);
  }
  ~Service() = default;

  std::shared_ptr<void> CreateRequest() {
    return std::shared_ptr<void>(new Cmd());
  }
  void HandleRequest(
      std::shared_ptr<MessageHeader> request_header,
      std::shared_ptr<void> request) {
    auto typed_request = std::static_pointer_cast<Cmd>(request);
    auto typed_response = std::shared_ptr<Ack>(new Ack);
    callback_.Dispatch(request_header, typed_request, typed_response);
    SendResponse(request_header, typed_response);
  }

  void SendResponse(
      std::shared_ptr<MessageHeader> request_header,
      std::shared_ptr<Ack> response) {
    bool ret = GetHandle()->GetProtocol()->SendResponse(GetCommandInfo().get(), request_header.get(), response.get());

    if (!ret) {
      DLOG_ERROR << "Send response failed!";
    }
  }

 private:
  ServiceCallback<Cmd, Ack> callback_;
};
}
#endif //ROBORTS_SDK_DISPATCH_H
