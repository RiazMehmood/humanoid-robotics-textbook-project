#!/usr/bin/env python3
"""Simple script to verify all routes are registered."""
import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

print("=" * 60)
print("Route Verification")
print("=" * 60)

try:
    from main import app
    
    # Get all routes
    all_routes = []
    for route in app.routes:
        if hasattr(route, 'path'):
            methods = list(route.methods) if hasattr(route, 'methods') else []
            all_routes.append((route.path, methods))
    
    print(f"\n📋 Total routes found: {len(all_routes)}\n")
    
    # Categorize routes
    chatbot_routes = [r for r in all_routes if 'chatbot' in r[0]]
    content_routes = [r for r in all_routes if 'content' in r[0] or 'admin' in r[0]]
    other_routes = [r for r in all_routes if 'chatbot' not in r[0] and 'content' not in r[0] and 'admin' not in r[0]]
    
    # Display chatbot routes
    if chatbot_routes:
        print("🤖 Chatbot Routes:")
        for path, methods in chatbot_routes:
            print(f"   ✅ {path:40} {methods}")
    else:
        print("❌ No chatbot routes found!")
    
    # Display content ingestion routes
    if content_routes:
        print("\n📚 Content Ingestion Routes:")
        for path, methods in content_routes:
            print(f"   ✅ {path:40} {methods}")
    else:
        print("\n❌ No content ingestion routes found!")
    
    # Display other routes
    if other_routes:
        print("\n📄 Other Routes:")
        for path, methods in other_routes:
            print(f"   ✅ {path:40} {methods}")
    
    # Summary
    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    
    success = True
    if not chatbot_routes:
        print("❌ Chatbot routes: NOT FOUND")
        success = False
    else:
        print(f"✅ Chatbot routes: {len(chatbot_routes)} route(s) found")
    
    if not content_routes:
        print("❌ Content ingestion routes: NOT FOUND")
        success = False
    else:
        print(f"✅ Content ingestion routes: {len(content_routes)} route(s) found")
    
    # Check for critical routes
    critical_routes = ['/chatbot/query']
    missing = []
    for route_path in critical_routes:
        if not any(r[0] == route_path for r in all_routes):
            missing.append(route_path)
    
    if missing:
        print(f"❌ Missing critical routes: {missing}")
        success = False
    else:
        print("✅ All critical routes are registered")
    
    print("=" * 60)
    
    if success:
        print("🎉 All routes verified successfully!")
        sys.exit(0)
    else:
        print("⚠️  Some routes are missing. Check the errors above.")
        sys.exit(1)
        
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)




