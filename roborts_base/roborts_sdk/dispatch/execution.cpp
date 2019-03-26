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

#include "execution.h"

namespace roborts_sdk {
Executor::Executor(std::shared_ptr<Handle> handle) : handle_(handle) {}
std::shared_ptr<Handle> Executor::GetHandle() {
  return handle_;
}

void Executor::ExecuteSubscription(const std::shared_ptr<SubscriptionBase>& subscription) {
  auto message_header = subscription->CreateMessageHeader();
  std::shared_ptr<void> message = subscription->CreateMessage();

  bool ret = GetHandle()->GetProtocol()->Take(
      subscription->GetCommandInfo().get(),
      message_header.get(),
      message.get());
  if (ret) {
    subscription->HandleMessage(message_header, message);
  } else {
//      DLOG_ERROR<<"take message failed!";
  }
  //TODO: add return message;
  //subscription->return_message(message);
}
void Executor::ExecuteService(const std::shared_ptr<ServiceBase>& service) {
  auto request_header = service->CreateRequestHeader();
  std::shared_ptr<void> request = service->CreateRequest();

  bool ret = GetHandle()->GetProtocol()->Take(
      service->GetCommandInfo().get(),
      request_header.get(),
      request.get());
  if (ret) {
    service->HandleRequest(request_header, request);
  } else {
    DLOG_ERROR << "take request failed!";
  }
}
void Executor::ExecuteClient(const std::shared_ptr<ClientBase>& client) {
  auto request_header = client->CreateRequestHeader();
  std::shared_ptr<void> response = client->CreateResponse();

  bool ret = GetHandle()->GetProtocol()->Take(
      client->GetCommandInfo().get(),
      request_header.get(),
      response.get());

  if (ret) {
    client->HandleResponse(request_header, response);
  } else {
//      DLOG_ERROR<<"take response failed!";
  }
}
}