#!/bin/bash

# Start All Services Script
# This script starts all three services required for the platform

echo "🚀 Starting Physical AI & Humanoid Robotics Platform..."
echo ""

# Check if required directories exist
if [ ! -d "backend" ] || [ ! -d "auth-server" ] || [ ! -d "physical-ai-humanoid-robotics" ]; then
    echo "❌ Error: Required directories not found. Are you in the project root?"
    exit 1
fi

# Check if .env files exist
if [ ! -f ".env" ]; then
    echo "⚠️  Warning: .env file not found. Copy .env.example to .env and configure it."
    exit 1
fi

if [ ! -f "auth-server/.env" ]; then
    echo "⚠️  Warning: auth-server/.env not found. Copy auth-server/.env.example to auth-server/.env"
    exit 1
fi

if [ ! -f "physical-ai-humanoid-robotics/.env.local" ]; then
    echo "⚠️  Warning: physical-ai-humanoid-robotics/.env.local not found."
    exit 1
fi

# Function to check if a port is in use
check_port() {
    if lsof -Pi :$1 -sTCP:LISTEN -t >/dev/null 2>&1 ; then
        echo "⚠️  Port $1 is already in use"
        return 1
    fi
    return 0
}

# Check ports
echo "🔍 Checking ports..."
check_port 8001 || exit 1
check_port 3001 || exit 1
check_port 3000 || exit 1

echo "✅ All ports available"
echo ""

# Create logs directory
mkdir -p logs

# Start Backend
echo "🔧 Starting Backend (Port 8001)..."
cd backend
if [ ! -d "venv" ]; then
    echo "⚠️  Virtual environment not found. Creating..."
    python -m venv venv
fi

source venv/bin/activate 2>/dev/null || source venv/Scripts/activate 2>/dev/null
python main.py > ../logs/backend.log 2>&1 &
BACKEND_PID=$!
echo "✅ Backend started (PID: $BACKEND_PID)"
cd ..

# Wait a bit for backend to initialize
sleep 2

# Start Auth Server
echo "🔐 Starting Auth Server (Port 3001)..."
cd auth-server
npm start > ../logs/auth.log 2>&1 &
AUTH_PID=$!
echo "✅ Auth Server started (PID: $AUTH_PID)"
cd ..

# Wait a bit for auth server to initialize
sleep 2

# Start Frontend
echo "🎨 Starting Frontend (Port 3000)..."
cd physical-ai-humanoid-robotics
npm start > ../logs/frontend.log 2>&1 &
FRONTEND_PID=$!
echo "✅ Frontend started (PID: $FRONTEND_PID)"
cd ..

echo ""
echo "🎉 All services started successfully!"
echo ""
echo "📊 Service Status:"
echo "   Backend:  http://localhost:8001 (PID: $BACKEND_PID)"
echo "   Auth:     http://localhost:3001 (PID: $AUTH_PID)"
echo "   Frontend: http://localhost:3000 (PID: $FRONTEND_PID)"
echo ""
echo "📝 Logs are available in the logs/ directory"
echo ""
echo "To stop all services, run: ./stop-all.sh"
echo "Or manually kill processes: kill $BACKEND_PID $AUTH_PID $FRONTEND_PID"
echo ""

# Save PIDs to file for stop script
echo "$BACKEND_PID" > logs/backend.pid
echo "$AUTH_PID" > logs/auth.pid
echo "$FRONTEND_PID" > logs/frontend.pid

echo "✨ Platform is ready! Open http://localhost:3000 in your browser"
