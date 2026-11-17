"""Vuer (0.0.32rc7) with an idle wait in its send loop.

Vuer's uplink polls each headset's queue with sleep(0): one CPU core at
100% per headset, competing with the camera thread for the GIL. Same
code, but it sleeps 1 ms when the queue is empty.
"""

from asyncio import sleep

from vuer import Vuer
from websockets import ConnectionClosedError

IDLE_WAIT = 0.001  # s; frames go out at 30 Hz, so 1 ms adds no visible delay


class TeleVisionVuer(Vuer):

    async def uplink(self, proxy):
        ws_id = proxy.CURRENT_WS_ID
        queue = proxy.uplink_queue
        while ws_id in self.ws:
            if not queue:
                await sleep(IDLE_WAIT)
                continue
            msg_bytes = queue.popleft()
            try:
                await self.send(ws_id, event_bytes=msg_bytes)
            except (ConnectionResetError, ConnectionClosedError):
                await self.close_ws(ws_id)
                return
            except Exception:
                await self.close_ws(ws_id)
                raise

    def get_url(self):
        # Vuer prints "Visit: https://vuer.ai...", its hosted page. The page
        # is served here instead: the node logs the address to open.
        return "the 'Open in the headset' address above"
