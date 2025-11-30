"""Content personalization service for user preferences and recommendations."""
from typing import List, Dict, Optional, Any
from datetime import datetime
from src.utils.logger import logger

# Optional SQLAlchemy imports
try:
    from sqlalchemy.orm import Session
    from src.models.user import User
    SQLALCHEMY_AVAILABLE = True
except ImportError:
    SQLALCHEMY_AVAILABLE = False
    Session = type('Session', (), {})
    User = None


class PersonalizationService:
    """Service for content personalization based on user preferences and interaction history."""
    
    def __init__(self):
        """Initialize the personalization service."""
        logger.info("PersonalizationService initialized")
    
    def get_user_preferences(
        self,
        db: Session,
        user_id: int
    ) -> Dict[str, Any]:
        """
        Get user content preferences.
        
        Args:
            db: Database session
            user_id: User ID
            
        Returns:
            Dictionary with user preferences
        """
        try:
            if SQLALCHEMY_AVAILABLE and User and hasattr(db, 'query'):
                user = db.query(User).filter(User.id == user_id).first()
                if user and user.preferences:
                    return user.preferences
            # Return default preferences if database not available or user not found
            return {
                "language": "en",
                "theme": "light"
            }
        except Exception as e:
            logger.error(f"Error getting user preferences: {e}")
            return {
                "language": "en",
                "theme": "light"
            }
    
    def update_user_preferences(
        self,
        db: Session,
        user_id: int,
        preferences: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Update user content preferences.
        
        Args:
            db: Database session
            user_id: User ID
            preferences: Dictionary with preference updates
            
        Returns:
            Updated preferences dictionary
        """
        try:
            if SQLALCHEMY_AVAILABLE and User and hasattr(db, 'add'):
                user = db.query(User).filter(User.id == user_id).first()
                if user:
                    # Merge with existing preferences
                    current_prefs = user.preferences or {}
                    current_prefs.update(preferences)
                    user.preferences = current_prefs
                    db.commit()
                    db.refresh(user)
                    logger.info(f"Updated preferences for user {user_id}")
                    return current_prefs
            # Return updated preferences even if database not available
            return preferences
        except Exception as e:
            logger.error(f"Error updating user preferences: {e}")
            if hasattr(db, 'rollback'):
                try:
                    db.rollback()
                except:
                    pass
            return preferences
    
    def get_content_recommendations(
        self,
        db: Session,
        user_id: int,
        limit: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Get personalized content recommendations based on user interaction history and background.
        
        Args:
            db: Database session
            user_id: User ID
            limit: Maximum number of recommendations
            
        Returns:
            List of recommended content items
        """
        try:
            recommendations = []
            
            if SQLALCHEMY_AVAILABLE and User and hasattr(db, 'query'):
                user = db.query(User).filter(User.id == user_id).first()
                if user:
                    # 1. Recommendations based on reading history
                    if user.interaction_history:
                        history = user.interaction_history
                        read_chapters = history.get("read_chapters", [])
                        
                        if read_chapters:
                            for chapter in read_chapters[-3:]:
                                recommendations.append({
                                    "contentId": f"{chapter}-next",
                                    "title": f"Continue reading after {chapter}",
                                    "type": "chapter",
                                    "reason": "Based on your reading history"
                                })

                    # 2. Recommendations based on background info
                    if len(recommendations) < limit and user.background_info:
                        bg = user.background_info
                        
                        # If novice in Robotics, recommend Intro
                        if bg.get("roboticsExp") == "novice":
                            recommendations.append({
                                "contentId": "module1-introduction",
                                "title": "Introduction to Robotics",
                                "type": "chapter",
                                "reason": "Recommended for robotics beginners"
                            })
                            
                        # If expert in Software but novice in Hardware, recommend Hardware modules
                        if bg.get("softwareExp") == "expert" and bg.get("hardwareExp") == "novice":
                             recommendations.append({
                                "contentId": "module2-sensor-simulation",
                                "title": "Hardware & Sensors",
                                "type": "chapter",
                                "reason": "Bridge your software skills to hardware"
                            })

                        # If expert in Python, recommend advanced Python agents
                        if "python" in str(bg.get("programmingLanguages", "")).lower():
                             recommendations.append({
                                "contentId": "module1-python-agents-ros",
                                "title": "Advanced Python Agents in ROS",
                                "type": "chapter",
                                "reason": "Leverage your Python expertise"
                            })

            # 3. Default recommendations if still not enough
            if len(recommendations) < limit:
                defaults = [
                    {
                        "contentId": "module1-introduction",
                        "title": "Introduction to Physical AI",
                        "type": "chapter",
                        "reason": "Popular starting point"
                    },
                    {
                        "contentId": "module2-gazebo-simulation",
                        "title": "Gazebo Simulation",
                        "type": "chapter",
                        "reason": "Hands-on simulation"
                    }
                ]
                # Add defaults that aren't already in recommendations
                existing_ids = {r["contentId"] for r in recommendations}
                for d in defaults:
                    if d["contentId"] not in existing_ids:
                        recommendations.append(d)
            
            return recommendations[:limit]
        except Exception as e:
            logger.error(f"Error getting content recommendations: {e}")
            return []
    
    def analyze_user_interaction(
        self,
        db: Session,
        user_id: int,
        interaction_type: str,
        content_id: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Record user interaction for personalization.
        
        Args:
            db: Database session
            user_id: User ID
            interaction_type: Type of interaction (e.g., "read", "query", "bookmark")
            content_id: ID of the content interacted with
            metadata: Additional metadata about the interaction
        """
        try:
            if SQLALCHEMY_AVAILABLE and User and hasattr(db, 'add'):
                user = db.query(User).filter(User.id == user_id).first()
                if user:
                    history = user.interaction_history or {}
                    
                    # Update interaction history based on type
                    if interaction_type == "read":
                        read_chapters = history.get("read_chapters", [])
                        if content_id not in read_chapters:
                            read_chapters.append(content_id)
                            history["read_chapters"] = read_chapters
                    elif interaction_type == "query":
                        queries = history.get("chatbot_queries", [])
                        queries.append({
                            "content_id": content_id,
                            "timestamp": datetime.utcnow().isoformat(),
                            "metadata": metadata or {}
                        })
                        history["chatbot_queries"] = queries
                    
                    user.interaction_history = history
                    db.commit()
                    logger.debug(f"Recorded {interaction_type} interaction for user {user_id}")
        except Exception as e:
            logger.error(f"Error analyzing user interaction: {e}")
            if hasattr(db, 'rollback'):
                try:
                    db.rollback()
                except:
                    pass


# Global personalization service instance
personalization_service = PersonalizationService()



