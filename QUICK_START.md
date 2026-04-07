# 🚀 Quick Start Guide

Get the Physical AI & Humanoid Robotics Platform running in 10 minutes!

## Prerequisites Check

Before starting, ensure you have:

```bash
# Check Python version (need 3.11+)
python --version

# Check Node.js version (need 20+)
node --version

# Check PostgreSQL (need for auth)
psql --version
```

If any are missing, install them first.

## Step 1: Clone and Setup (2 minutes)

```bash
# Clone the repository
git clone https://github.com/yourusername/physical-ai-humanoid-robotics-book.git
cd physical-ai-humanoid-robotics-book

# Copy environment template
cp .env.example .env
```

## Step 2: Configure Environment (3 minutes)

Edit `.env` file with your credentials:

```env
# Get free Qdrant cloud instance at https://cloud.qdrant.io
QDRANT_URL=https://your-instance.cloud.qdrant.io
QDRANT_API_KEY=your_api_key_here

# Get OpenRouter API key at https://openrouter.ai
OPENROUTER_API_KEY=your_openrouter_key_here

# Optional: adjust similarity threshold (0.0-1.0)
RAG_SIMILARITY_THRESHOLD=0.6
```

Configure auth server:

```bash
# Create auth-server/.env
cat > auth-server/.env << EOF
DATABASE_URL=postgresql://postgres:password@localhost:5432/auth_db
BETTER_AUTH_SECRET=$(openssl rand -hex 32)
BETTER_AUTH_URL=http://localhost:3001
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001
EOF
```

Configure frontend:

```bash
# Create physical-ai-humanoid-robotics/.env.local
cat > physical-ai-humanoid-robotics/.env.local << EOF
BETTER_AUTH_URL=http://localhost:3001
CHATKIT_API_URL=http://localhost:8001/chatkit
EOF
```

## Step 3: Install Dependencies (3 minutes)

### Backend
```bash
cd backend
python -m venv venv

# Windows
venv\Scripts\activate

# Linux/Mac
source venv/bin/activate

pip install -r requirements.txt
cd ..
```

### Frontend
```bash
cd physical-ai-humanoid-robotics
npm install
cd ..
```

### Auth Server
```bash
cd auth-server
npm install
node migrate.js  # Setup database
cd ..
```

## Step 4: Generate Embeddings (2 minutes)

```bash
# This creates vector embeddings from book content
python generate_book_embeddings.py
```

Wait for completion. You should see:
```
✓ Processed X files
✓ Generated Y embeddings
✓ Uploaded to Qdrant
```

## Step 5: Start Services (1 minute)

Open **three terminal windows**:

### Terminal 1: Backend
```bash
cd backend
python main.py
```
Wait for: `Starting ChatKit Gemini server at http://localhost:8001`

### Terminal 2: Auth Server
```bash
cd auth-server
npm start
```
Wait for: `Better Auth server running on port 3001`

### Terminal 3: Frontend
```bash
cd physical-ai-humanoid-robotics
npm start
```
Wait for browser to open at `http://localhost:3000`

## Step 6: Test It! (1 minute)

1. **Open** `http://localhost:3000` in your browser
2. **Click** the chatbot icon (bottom right)
3. **Type** a question: "What is ROS2?"
4. **Watch** the AI respond with book content!

## ✅ Success Checklist

- [ ] All three services running without errors
- [ ] Frontend loads at localhost:3000
- [ ] Chatbot widget appears
- [ ] Can send messages and get responses
- [ ] Responses reference book content

## 🐛 Quick Troubleshooting

### Backend won't start
```bash
# Check if port is in use
netstat -ano | findstr :8001  # Windows
lsof -i :8001                 # Mac/Linux

# Kill the process and restart
```

### "No module named 'qdrant_client'"
```bash
cd backend
pip install -r requirements.txt
```

### Chatbot not responding
1. Check all three terminals for errors
2. Verify `.env` files are configured
3. Check browser console (F12) for errors
4. Ensure embeddings were generated

### Auth errors
```bash
cd auth-server
node migrate.js  # Re-run migration
npm start
```

### Frontend build errors
```bash
cd physical-ai-humanoid-robotics
rm -rf node_modules package-lock.json
npm install
npm run clear
npm start
```

## 🎯 Next Steps

Now that it's running:

1. **Explore the Docs**: Browse the documentation pages
2. **Ask Questions**: Try different robotics questions
3. **Create Account**: Sign up to save conversations
4. **Read the Code**: Check out the architecture
5. **Customize**: Modify the chatbot instructions

## 📚 Learn More

- [Full README](./README.md) - Complete documentation
- [Architecture](./README.md#-architecture) - System design
- [API Docs](./README.md#-api-endpoints) - Endpoint reference
- [Testing](./TESTING-GUIDE.md) - Testing guide

## 💡 Pro Tips

### Development Workflow
```bash
# Use tmux/screen to manage multiple terminals
tmux new -s robotics
# Split panes: Ctrl+B then %
# Switch panes: Ctrl+B then arrow keys
```

### Hot Reload
- Backend: Restart manually after changes
- Frontend: Auto-reloads on save
- Auth: Restart manually after changes

### Debug Mode
```bash
# Backend with debug logging
cd backend
python -c "import logging; logging.basicConfig(level=logging.DEBUG)"
python main.py
```

### Quick Reset
```bash
# Reset everything
pkill -f "python main.py"
pkill -f "node server.js"
pkill -f "docusaurus start"

# Restart all services
./start-all.sh  # If you create this script
```

## 🎉 You're Ready!

You now have a fully functional AI-powered robotics learning platform!

**Questions?** Check the [FAQ](./README.md#-faq) or reach out on [LinkedIn](https://www.linkedin.com/in/muskan-muhammad-atiq-agentic-ai-expert).

---

**Happy Learning! 🤖**
