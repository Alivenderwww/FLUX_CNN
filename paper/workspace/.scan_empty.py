import sys
with open(r'c:\_Project\FLUX_CNN\paper\workspace\paper.md', encoding='utf-8') as f:
    lines = f.readlines()

def line_kind(l):
    t = l.rstrip('\n').strip()
    if t == '': return 'empty'
    if t.startswith('#'): return 'heading'
    if t.startswith('|'): return 'table'
    if t.startswith('---'): return 'hr'
    if t.startswith('~~~') or t.startswith('```'): return 'codefence'
    if t.startswith('>'): return 'blockquote'
    if t.startswith('- ') or t.startswith('* ') or t.startswith('+ '): return 'list'
    if t.startswith('<!--'): return 'comment'
    if t.startswith('**Figure ') or t.startswith('**Table ') or t.startswith('**图 ') or t.startswith('**表 '): return 'caption'
    if t.startswith('Figure ') or t.startswith('Table ') or t.startswith('表 ') or t.startswith('图 '): return 'caption'
    return 'para'

kinds = [line_kind(l) for l in lines]
cands = []
for i in range(454, len(lines)-1):
    if kinds[i] == 'empty':
        pk = kinds[i-1] if i>0 else None
        nk = kinds[i+1] if i+1<len(lines) else None
        if pk == 'para' and nk == 'para':
            cands.append(i+1)
print('count:', len(cands))
print('lines:', cands)
