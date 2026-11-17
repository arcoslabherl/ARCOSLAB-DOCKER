from aiohttp.test_utils import make_mocked_request

from television_teleop.vuer_server import ws_redirect


def request(path, scheme='https', host='tower.tail219772.ts.net:8012', headers=None):
    req = make_mocked_request('GET', path, headers=headers)
    return req.clone(scheme=scheme, host=host)


def test_page_without_ws_redirects_to_same_host():
    url = ws_redirect(request('/'))
    assert str(url) == '/?ws=wss://tower.tail219772.ts.net:8012'


def test_keeps_other_query_values():
    url = ws_redirect(request('/?grid=False'))
    assert url.query['grid'] == 'False'
    assert url.query['ws'] == 'wss://tower.tail219772.ts.net:8012'


def test_plain_http_gives_ws():
    url = ws_redirect(request('/', scheme='http', host='localhost:8012'))
    assert url.query['ws'] == 'ws://localhost:8012'


def test_no_redirect_with_ws_or_for_the_websocket():
    assert ws_redirect(request('/?ws=wss://x:8012')) is None
    assert ws_redirect(request('/', headers={'Upgrade': 'websocket'})) is None
