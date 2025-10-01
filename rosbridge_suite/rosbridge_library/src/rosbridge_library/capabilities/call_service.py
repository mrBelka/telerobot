# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import annotations

import fnmatch
from functools import partial
from threading import Thread
from typing import TYPE_CHECKING, Any

from rosbridge_library.capability import Capability
from rosbridge_library.internal.services import ServiceCaller

if TYPE_CHECKING:
    from rosbridge_library.protocol import Protocol


class CallService(Capability):
    call_service_msg_fields = (
        (True, "service", str),
        (False, "fragment_size", (int, type(None))),
        (False, "compression", str),
    )

    services_glob: list[str] | None = None

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        self.default_timeout = (
            protocol.node_handle.get_parameter("default_call_service_timeout")
            .get_parameter_value()
            .double_value
        )

        # Register the operations that this capability provides
        call_services_in_new_thread = (
            protocol.node_handle.get_parameter("call_services_in_new_thread")
            .get_parameter_value()
            .bool_value
        )
        if call_services_in_new_thread:
            # Calls the service in a separate thread so multiple services can be processed simultaneously.
            protocol.node_handle.get_logger().info("Calling services in new thread")
            protocol.register_operation(
                "call_service", lambda msg: Thread(target=self.call_service, args=(msg,)).start()
            )
        else:
            # Calls the service in this thread, so services block and must be processed sequentially.
            protocol.node_handle.get_logger().info("Calling services in existing thread")
            protocol.register_operation("call_service", self.call_service)

    def call_service(self, message: dict[str, Any]) -> None:
        # Pull out the ID
        cid: str | None = message.get("id")

        # Typecheck the args
        self.basic_type_check(message, self.call_service_msg_fields)

        # Extract the args
        service: str = message["service"]
        fragment_size: int | None = message.get("fragment_size")
        compression: str = message.get("compression", "none")
        args: list | dict[str, Any] = message.get("args", [])
        timeout: float = message.get("timeout", self.default_timeout)

        if CallService.services_glob is not None and CallService.services_glob:
            self.protocol.log(
                "debug", "Service security glob enabled, checking service: " + service
            )
            match = False
            for glob in CallService.services_glob:
                if fnmatch.fnmatch(service, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", continuing service call...",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for service, cancelling service call for: " + service,
                )
                return
        else:
            self.protocol.log("debug", "No service security glob, not checking service call.")

        # Check for deprecated service ID, eg. /rosbridge/topics#33
        cid = extract_id(service, cid)

        # Create the callbacks
        s_cb = partial(self._success, cid, service, fragment_size, compression)
        e_cb = partial(self._failure, cid, service)

        # Run service caller in the same thread.
        ServiceCaller(
            trim_servicename(service),
            args,
            timeout,
            s_cb,
            e_cb,
            self.protocol.node_handle,
        ).run()

    def _success(
        self,
        cid: str | None,
        service: str,
        _fragment_size: int | None,
        _compression: str,
        message: dict[str, Any],
    ) -> None:
        outgoing_message = {
            "op": "service_response",
            "service": service,
            "values": message,
            "result": True,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def _failure(self, cid: str | None, service: str, exc: Exception) -> None:
        self.protocol.log("error", f"call_service {type(exc).__name__}: {exc!s}", cid)
        # send response with result: false
        outgoing_message = {
            "op": "service_response",
            "service": service,
            "values": str(exc),
            "result": False,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        self.protocol.send(outgoing_message)


def trim_servicename(service: str) -> str:
    if "#" in service:
        return service[: service.find("#")]
    return service


def extract_id(service: str, cid: str | None) -> str | None:
    if cid is not None:
        return cid
    if "#" in service:
        return service[service.find("#") + 1 :]
    return None
