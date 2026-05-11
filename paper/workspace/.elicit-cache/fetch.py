import urllib.request as u, json, sys, os, time, hashlib

KEY = open(r'c:/_Project/FLUX_CNN/paper/.env').read().split('ELICIT_API_KEY=')[1].split()[0]
OUTDIR = r'c:/_Project/FLUX_CNN/paper/workspace/.elicit-cache'

QUERIES = [
    ('a_systolic',  'systolic array CNN accelerator weight stationary 2D MAC'),
    ('b_dataflow',  'CNN dataflow row-stationary output-stationary PE utilization'),
    ('c_streaming', 'streaming CNN accelerator FPGA line buffer row buffer'),
    ('d_pe_util',   'CNN accelerator PE utilization small channel kernel folding space-to-depth'),
    ('e_axi_dma',   'FPGA CNN accelerator AXI DMA DataMover descriptor'),
    ('f_compiler',  'CNN accelerator compiler tiling slicing channel splitting'),
    ('g_int8_sdp',  'INT8 CNN accelerator quantization bias residual fusion ResNet'),
    ('h_arbitrary', 'CNN accelerator arbitrary feature map size streaming row ring buffer FPGA'),
]

def fetch(name, query, max_results=20):
    out = os.path.join(OUTDIR, f'{name}.json')
    if os.path.exists(out) and os.path.getsize(out) > 100:
        print(f'SKIP {name}: cached')
        return
    body = json.dumps({'query': query, 'maxResults': max_results, 'searchMode': 'semantic'}).encode()
    req = u.Request('https://api.elicit.com/api/v1/search', data=body,
                    headers={'Authorization': 'Bearer ' + KEY, 'Content-Type': 'application/json'})
    try:
        r = u.urlopen(req, timeout=120)
        data = r.read()
        with open(out, 'wb') as f:
            f.write(data)
        print(f'OK   {name}: {len(data)} bytes, status {r.status}')
    except Exception as e:
        print(f'FAIL {name}: {e}')
    time.sleep(2)

for name, q in QUERIES:
    fetch(name, q, 20)
print('DONE')
