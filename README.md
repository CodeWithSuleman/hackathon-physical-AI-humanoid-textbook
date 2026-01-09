# 🤖 Physical AI Humanoid Textbook

An **AI-powered interactive humanoid textbook platform** that combines a **notebook-style physical AI book**, a **React-based RAG chatbot**, and an **agentic AI backend** to simulate a real-world Physical AI learning experience.

This project was built for a **hackathon**, focusing on modern AI tooling, retrieval-augmented generation, and human-like interaction with educational content.

---

## 📌 Project Idea (High-Level)

The core idea of this project is to create a **Physical AI Book** — a notebook-style digital textbook about Physical AI — enhanced with an **intelligent chatboard** that allows users to ask questions directly from the book content.

- 📘 The **book** is built using **Docusaurus**
- 💬 A **React-based chatboard** is embedded alongside the book
- 🧠 The chatboard uses **RAG (Retrieval-Augmented Generation)**
- 🤖 The backend is agent-driven using **OpenAI Agents SDK**
- 🔗 Models are served via **OpenRouter**
- 📦 Knowledge is stored using **Qdrant + Cohere embeddings**

---

## ✨ Key Features

### Frontend (Physical AI Book + Chatboard)
- 📘 Docusaurus-based **Physical AI Notebook**
- 🧩 Structured chapters and learning content
- 💬 Embedded **React RAG Chatboard**
- 🔄 Real-time question answering from book content
- 🎯 Context-aware responses linked to textbook material

### Backend (Agentic AI)
- 🤖 Agent-based backend using OpenAI Agents SDK
- 🧠 RAG pipeline with semantic search
- 🔎 Qdrant vector database
- 📐 Cohere embeddings
- 🌐 OpenRouter-hosted LLMs

---

## 🧰 Tech Stack

### 📘 Frontend (Book + UI)
- Docusaurus (Physical AI Notebook)
- React
- JavaScript / TypeScript
- Markdown-based content system

### 🧠 Backend
- Python
- FastAPI
- OpenAI Agents SDK
- OpenRouter API
- Qdrant Vector Database
- Cohere Embeddings

### 🧪 Tooling
- Gemini CLI
- Notebook-based data preparation
- Prompt engineering for agents

---

## 📘 Frontend Explained (Very Important)

### 🧠 Physical AI Notebook (Docusaurus)

The **Physical AI Book** is built using **Docusaurus**, which acts as a structured digital notebook:

- Chapters written in Markdown
- Topics focused on Physical AI concepts
- Acts as the **knowledge source** for the RAG system
- Easy to extend and maintain

This makes the project feel like a **real textbook**, not just a chatbot.

---

### 💬 React RAG Chatboard

Alongside the notebook, a **React-based chatboard** is integrated:

- Users ask questions while reading the book
- Queries are sent to the backend
- Relevant content is retrieved from the book embeddings
- Answers are generated using LLMs with context

The chatboard transforms the book into an **interactive humanoid tutor**.

---

## 🧠 RAG + Agent Flow

