import React, { useState, useRef, useEffect } from 'react';
import axios from 'axios';

const HuggingFaceChat = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [input, setInput] = useState('');
  const [messages, setMessages] = useState([
    { role: 'bot', content: 'Welcome to the Physical AI Lab! 🤖 I am your assistant. How can I help you today?' }
  ]);
  const [loading, setLoading] = useState(false);
  const chatEndRef = useRef(null);

  const scrollToBottom = () => chatEndRef.current?.scrollIntoView({ behavior: "smooth" });
  useEffect(() => { scrollToBottom(); }, [messages]);

  const handleSend = async (e) => {
    e.preventDefault();
    if (!input.trim()) return;

    const userMsg = { role: 'user', content: input };
    setMessages([...messages, userMsg]);
    setInput('');
    setLoading(true);

    try {
      const res = await axios.post('https://suleman-shaikh-rag-chatbot.hf.space/query', {
        question: input 
      });
      const botMsg = { role: 'bot', content: res.data.answer };
      setMessages((prev) => [...prev, botMsg]);
    } catch (err) {
      setMessages((prev) => [...prev, { role: 'bot', content: '⚠️ Connection lost. Please check your internet.' }]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="bot-container">
      {/* 1. Animated Floating Button */}
      <button 
        onClick={() => setIsOpen(!isOpen)} 
        className={`floating-btn ${isOpen ? 'active' : ''}`}
      >
        {isOpen ? '✕' : <img src="https://cdn-icons-png.flaticon.com/512/4712/4712035.png" alt="bot" />}
      </button>

      {/* 2. Responsive Chat Window */}
      <div className={`chat-window ${isOpen ? 'open' : ''}`}>
        <div className="chat-header">
          <div className="header-info">
            <div className="online-indicator"></div>
            <div>
              <div className="bot-name">Physical AI Guide</div>
              <div className="bot-status">AI Assistant Online</div>
            </div>
          </div>
        </div>
        
        <div className="chat-body">
          {messages.map((m, i) => (
            <div key={i} className={`message-wrapper ${m.role}`}>
              <div className="message-bubble">{m.content}</div>
            </div>
          ))}
          {loading && (
            <div className="message-wrapper bot">
              <div className="message-bubble loading-dots">Thinking<span>.</span><span>.</span><span>.</span></div>
            </div>
          )}
          <div ref={chatEndRef} />
        </div>

        <form onSubmit={handleSend} className="chat-input-area">
          <input 
            value={input} 
            onChange={(e) => setInput(e.target.value)} 
            placeholder="Ask anything..." 
          />
          <button type="submit">➤</button>
        </form>
      </div>

      <style>{`
        /* Global Variables for Theme */
        :root {
          --primary-color: #25c2a0;
          --dark-bg: #1b1b1d;
          --light-gray: #f4f7f6;
          --glass-effect: rgba(255, 255, 255, 0.9);
        }

        .bot-container {
          position: fixed;
          bottom: 25px;
          right: 25px;
          z-index: 999999;
          font-family: 'Inter', sans-serif;
        }

        /* Floating Button with Pulse & Spin */
        .floating-btn {
          width: 65px; height: 65px;
          border-radius: 50%;
          background: var(--primary-color);
          border: none;
          cursor: pointer;
          box-shadow: 0 8px 24px rgba(37, 194, 160, 0.4);
          transition: all 0.3s cubic-bezier(0.175, 0.885, 0.32, 1.275);
          display: flex; align-items: center; justify-content: center;
          color: white; font-size: 24px;
        }
        .floating-btn img { width: 35px; transition: 0.3s; }
        .floating-btn:hover { transform: scale(1.1) rotate(5deg); }
        .floating-btn.active { transform: rotate(90deg); background: #333; }

        /* Main Window with Responsive Design */
        .chat-window {
          position: absolute;
          bottom: 80px;
          right: 0;
          width: 380px;
          max-width: 90vw; /* Responsive for mobile */
          height: 600px;
          max-height: 70vh; /* Responsive for mobile */
          background: white;
          border-radius: 20px;
          display: flex;
          flex-direction: column;
          overflow: hidden;
          box-shadow: 0 20px 40px rgba(0,0,0,0.2);
          opacity: 0;
          transform: translateY(30px) scale(0.9);
          pointer-events: none;
          transition: all 0.4s cubic-bezier(0.165, 0.84, 0.44, 1);
        }
        .chat-window.open {
          opacity: 1;
          transform: translateY(0) scale(1);
          pointer-events: auto;
        }

        /* Header Style */
        .chat-header {
          background: var(--dark-bg);
          padding: 20px;
          color: white;
          border-bottom: 3px solid var(--primary-color);
        }
        .header-info { display: flex; align-items: center; gap: 12px; }
        .online-indicator {
          width: 10px; height: 10px;
          background: #4caf50;
          border-radius: 50%;
          animation: blink 1.5s infinite;
        }
        .bot-name { font-weight: 700; font-size: 16px; }
        .bot-status { font-size: 11px; color: #aaa; }

        /* Chat Bubbles & Animations */
        .chat-body {
          flex: 1;
          padding: 20px;
          overflow-y: auto;
          background: var(--light-gray);
          display: flex;
          flex-direction: column;
          gap: 15px;
        }
        .message-wrapper {
          display: flex;
          width: 100%;
          animation: fadeIn 0.3s ease forwards;
        }
        .message-wrapper.user { justify-content: flex-end; }
        .message-wrapper.bot { justify-content: flex-start; }

        .message-bubble {
          max-width: 80%;
          padding: 12px 16px;
          font-size: 14px;
          line-height: 1.5;
          border-radius: 18px;
        }
        .user .message-bubble {
          background: var(--primary-color);
          color: white;
          border-bottom-right-radius: 2px;
        }
        .bot .message-bubble {
          background: white;
          color: #333;
          border-bottom-left-radius: 2px;
          box-shadow: 0 2px 8px rgba(0,0,0,0.05);
        }

        /* Input Area Style */
        .chat-input-area {
          padding: 15px;
          background: white;
          display: flex;
          gap: 10px;
          border-top: 1px solid #eee;
        }
        .chat-input-area input {
          flex: 1;
          border: 1px solid #ddd;
          padding: 10px 15px;
          border-radius: 25px;
          outline: none;
          transition: 0.3s;
        }
        .chat-input-area input:focus { border-color: var(--primary-color); }
        .chat-input-area button {
          background: var(--primary-color);
          color: white;
          border: none;
          width: 40px; height: 40px;
          border-radius: 50%;
          cursor: pointer;
          transition: 0.3s;
        }
        .chat-input-area button:hover { transform: scale(1.1); }

        /* Keyframes */
        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(10px); }
          to { opacity: 1; transform: translateY(0); }
        }
        @keyframes blink {
          0% { opacity: 1; } 50% { opacity: 0.4; } 100% { opacity: 1; }
        }
        .loading-dots span {
          animation: loading 1.4s infinite;
          display: inline-block;
        }
        .loading-dots span:nth-child(2) { animation-delay: 0.2s; }
        .loading-dots span:nth-child(3) { animation-delay: 0.4s; }
        @keyframes loading {
          0% { opacity: .2; } 20% { opacity: 1; } 100% { opacity: .2; }
        }

        /* Responsive Mobile Adjustments */
        @media (max-width: 480px) {
          .bot-container { right: 15px; bottom: 15px; }
          .chat-window {
            width: calc(100vw - 30px);
            height: 80vh;
            right: 0;
          }
        }
      `}</style>
    </div>
  );
};

export default HuggingFaceChat;