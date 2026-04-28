"""
Test script to verify Hugging Face Spaces deployment.
Run this after deploying to test all endpoints.
"""

import requests
import json
import sys

# Replace with your actual Hugging Face Space URL
HF_SPACE_URL = "https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space"

def test_health():
    """Test health endpoint"""
    print("🔍 Testing health endpoint...")
    try:
        response = requests.get(f"{HF_SPACE_URL}/health", timeout=10)
        if response.status_code == 200:
            print("✅ Health check passed")
            print(f"   Response: {response.json()}")
            return True
        else:
            print(f"❌ Health check failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Health check error: {e}")
        return False

def test_root():
    """Test root endpoint"""
    print("\n🔍 Testing root endpoint...")
    try:
        response = requests.get(f"{HF_SPACE_URL}/", timeout=10)
        if response.status_code == 200:
            print("✅ Root endpoint passed")
            print(f"   Response: {response.json()}")
            return True
        else:
            print(f"❌ Root endpoint failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Root endpoint error: {e}")
        return False

def test_chat_health():
    """Test chat health endpoint"""
    print("\n🔍 Testing chat health endpoint...")
    try:
        response = requests.get(f"{HF_SPACE_URL}/api/v1/chat/health", timeout=10)
        if response.status_code == 200:
            print("✅ Chat health check passed")
            print(f"   Response: {response.json()}")
            return True
        else:
            print(f"❌ Chat health check failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Chat health check error: {e}")
        return False

def test_chat():
    """Test chat endpoint with streaming"""
    print("\n🔍 Testing chat endpoint...")
    try:
        response = requests.post(
            f"{HF_SPACE_URL}/api/v1/chat",
            json={"message": "What is ROS?"},
            headers={"Content-Type": "application/json"},
            stream=True,
            timeout=30
        )

        if response.status_code == 200:
            print("✅ Chat endpoint responding")
            print("   Streaming response:")

            # Read first few chunks
            chunk_count = 0
            for line in response.iter_lines():
                if line:
                    chunk_count += 1
                    decoded_line = line.decode('utf-8')
                    if decoded_line.startswith('data: '):
                        data = decoded_line[6:]
                        if data.strip() != '[DONE]':
                            try:
                                parsed = json.loads(data)
                                content = parsed.get('choices', [{}])[0].get('delta', {}).get('content', '')
                                if content:
                                    print(f"   {content}", end='', flush=True)
                            except:
                                pass

                    # Only show first 10 chunks for testing
                    if chunk_count >= 10:
                        print("\n   ... (truncated)")
                        break

            print("\n✅ Chat streaming works")
            return True
        else:
            print(f"❌ Chat endpoint failed: {response.status_code}")
            print(f"   Response: {response.text}")
            return False
    except Exception as e:
        print(f"❌ Chat endpoint error: {e}")
        return False

def test_docs():
    """Test API documentation endpoint"""
    print("\n🔍 Testing API docs endpoint...")
    try:
        response = requests.get(f"{HF_SPACE_URL}/docs", timeout=10)
        if response.status_code == 200:
            print("✅ API docs accessible")
            print(f"   URL: {HF_SPACE_URL}/docs")
            return True
        else:
            print(f"❌ API docs failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ API docs error: {e}")
        return False

def main():
    """Run all tests"""
    print("🚀 Hugging Face Spaces Deployment Test")
    print("=" * 50)
    print(f"Testing: {HF_SPACE_URL}")
    print("=" * 50)

    if HF_SPACE_URL == "https://YOUR_USERNAME-physical-ai-humanoid-robotics-backend.hf.space":
        print("\n⚠️  Please update HF_SPACE_URL in this script with your actual Space URL")
        sys.exit(1)

    results = []
    results.append(("Health", test_health()))
    results.append(("Root", test_root()))
    results.append(("Chat Health", test_chat_health()))
    results.append(("Chat", test_chat()))
    results.append(("Docs", test_docs()))

    print("\n" + "=" * 50)
    print("📊 Test Results Summary")
    print("=" * 50)

    for name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status} - {name}")

    total = len(results)
    passed = sum(1 for _, p in results if p)

    print(f"\nTotal: {passed}/{total} tests passed")

    if passed == total:
        print("\n🎉 All tests passed! Your deployment is working correctly.")
        return 0
    else:
        print("\n⚠️  Some tests failed. Check the logs above for details.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
