// 最小静态文件服务器（仅用于本地预览/截图，绑定 127.0.0.1）
const http = require('http');
const fs = require('fs');
const path = require('path');

const ROOT = 'F:/airace';
const PORT = 8099;
const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.png': 'image/png',
  '.svg': 'image/svg+xml',
  '.md': 'text/markdown; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
};

http
  .createServer((req, res) => {
    let p = decodeURIComponent(req.url.split('?')[0]);
    if (p === '/') p = '/四足机器狗导航_顶刊方案思维导图.html';
    const file = path.join(ROOT, p);
    if (!file.startsWith(path.resolve(ROOT))) {
      res.writeHead(403).end('forbidden');
      return;
    }
    fs.readFile(file, (err, buf) => {
      if (err) {
        res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' }).end('404 ' + p);
        return;
      }
      res.writeHead(200, { 'Content-Type': TYPES[path.extname(file)] || 'application/octet-stream' });
      res.end(buf);
    });
  })
  .listen(PORT, '127.0.0.1', () => console.log('serving http://127.0.0.1:' + PORT + '/'));