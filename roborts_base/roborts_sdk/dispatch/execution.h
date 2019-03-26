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

#ifndef ROBORTS_SDK_EXECUTION_H
#define ROBORTS_SDK_EXECUTION_H
#include "dispatch.h"

namespace roborts_sdk {
class Handle;
class SubscriptionBase;
class PublisherBase;
class ClientBase;
class ServiceBase;

/**
 * @brief Executor class to provide execute interface for subscriber, service server and client in the dispatch layer
 */
class Executor {
 public:
  /**
   * @brief Constructor of Executor
   * @param handle Pointer of handle which consists of handler of executor, protocol layer and hardware layer
   */
  Executor(std::shared_ptr<Handle> handle);
  ~Executor() = default;
  /**
   * @brief Get the handle
   * @return Pointer of handle
   */
  std::shared_ptr<Handle> GetHandle();
  /**
   * @brief Given the subscription, invoke its callback function
   * @param subscription Subscription base pointer of certain command
   */
  void ExecuteSubscription(const std::shared_ptr<SubscriptionBase>& subscription);
  /**
   * @brief Given the service, invoke its callback function and call the protocol layer to send response/ack
   * @param service Service base pointer of certain command
   */
  void ExecuteService(const std::shared_ptr<ServiceBase>& service);
  /**
   * @brief Given the client, call the protocol layer to send command and wait for the response/ack
   * @param client Client base pointer of certain command
   */
  void ExecuteClient(const std::shared_ptr<ClientBase>& client);

 private:
  //! pointer of handle
  std::shared_ptr<Handle> handle_;
};
}
#endif //ROBORTS_SDK_EXECUTION_H
