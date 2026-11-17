"""Vuer (0.0.32rc7) with two changes:

- Its send loop polls each headset's queue with sleep(0): one CPU core at
  100% per headset, competing with the camera thread for the GIL. Same
  code, but it sleeps 1 ms when the queue is empty.
- Its page connects to ?ws=<address>, or to ws://localhost:8012 without
  it (the headset itself). Opening the page without ?ws= redirects to the
  same address with it, so the headset only needs https://<machine>:8012.
"""

from asyncio import sleep

from aiohttp import web
from aiohttp.hdrs import UPGRADE
from vuer import Vuer
from websockets import ConnectionClosedError

IDLE_WAIT = 0.001  # s; frames go out at 30 Hz, so 1 ms adds no visible delay


def ws_redirect(request):
    """The page's address with ?ws= added, or None if it has it already
    (or is the websocket itself)."""
    if request.headers.get(UPGRADE, '').lower().strip() == 'websocket':
        return None
    if 'ws' in request.query:
        return None
    scheme = 'wss' if request.secure else 'ws'
    query = dict(request.query, ws=f'{scheme}://{request.host}')
    return request.rel_url.with_query(query)


class TeleVisionVuer(Vuer):

    async def socket_index(self, request):
        location = ws_redirect(request)
        if location is not None:
            raise web.HTTPFound(location)
        return await super().socket_index(request)

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
