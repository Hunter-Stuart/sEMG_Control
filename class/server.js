const net = require("net");
const WebSocket = require("ws");

// 创建TCP服务器
const tcpServer = net.createServer((socket) => {
  console.log("ESP01已连接");

  // 通知前端WiFi模块已连接
  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send("esp_connected");
    }
  });

  // 接收ESP01的数据
  socket.on("data", (data) => {
    const rawData = data.toString().trim();
    console.log("收到ESP01数据：", rawData);

    // 转发给所有连接的WebSocket客户端
    wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(rawData);
      }
    });
  });

  // TCP断开连接
  socket.on("close", () => {
    console.log("ESP01断开连接");
    // 通知网页断开
    wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send("esp_disconnected");
      }
    });
  });

  socket.on("error", (err) => {
    console.error("TCP错误：", err);
  });
});

// 创建WebSocket服务器
const wss = new WebSocket.Server({ port: 8080 });
wss.on("connection", (ws) => {
  console.log("网页已连接");
  ws.on("close", () => console.log("网页断开连接"));
  ws.on("error", (err) => console.error("WebSocket错误：", err));
});

// 启动TCP服务器
const TCP_PORT = 8888;
tcpServer.listen(TCP_PORT, () => {
  console.log(`TCP服务器启动，监听端口 ${TCP_PORT}`);
  console.log(`WebSocket服务器启动，端口 8080`);
});
