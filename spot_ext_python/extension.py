# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio
import gc

import carb
import omni.ext
import omni.physx
import omni.timeline

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation

SPOT_PATH = "/World/spot_with_arm"

def log(msg):
    carb.log_warn(f"[spot-ext] {msg}")

class Extension(omni.ext.IExt):

    def on_startup(self, ext_id: str):
        log("startup")
        self.world = World()
        self.world.reset()

        self.spot = Articulation(SPOT_PATH)
        self.spot.initialize()

        stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(self._on_timeline_event)
    
    def on_shutdown(self):
        log("shutdown")
        self.spot = None
        self.world = None
        self._timeline_sub = None
        self._inited = False

    def _on_timeline_event(self, event):
        if omni.timeline.get_timeline_interface().is_playing():
            asyncio.ensure_future(self._init_robot())

    async def _init_robot(self):
        if getattr(self, "_inited", False):
            return
        await omni.kit.app.get_app().next_update_async()
        self.world.reset()
        self.spot.initialize()

        log(f"DOF={self.spot.num_dof}")
        self._inited = True
