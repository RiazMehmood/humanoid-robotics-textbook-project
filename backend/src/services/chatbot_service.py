"""AI chatbot core service logic with Vector DB and LLM integration."""
from typing import List, Dict, Optional, Any
from datetime import datetime
from src.config import settings
from src.utils.logger import logger
from src.utils.errors import ValidationError

# Optional Google Generative AI import
try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError as e:
    GEMINI_AVAILABLE = False
    genai = None
    logger.warning(f"Google Generative AI not available: {e}")

# Optional SQLAlchemy imports
try:
    from sqlalchemy.orm import Session
    from src.models.chatbot_query import ChatbotQuery
    from src.models.chatbot_response import ChatbotResponse
    SQLALCHEMY_AVAILABLE = True
except ImportError:
    SQLALCHEMY_AVAILABLE = False
    Session = type('Session', (), {})
    ChatbotQuery = None
    ChatbotResponse = None

# Vector DB - optional, lazy initialization
try:
    from src.database.vector_db import get_vector_db
    VECTOR_DB_AVAILABLE = True
except ImportError:
    VECTOR_DB_AVAILABLE = False
    def get_vector_db():
        from unittest.mock import MagicMock
        mock = MagicMock()
        mock.search = lambda *args, **kwargs: []
        return mock


class ChatbotService:
    """Service for AI chatbot interactions using RAG (Retrieval-Augmented Generation)."""
    
    def __init__(self):
        """Initialize the chatbot service with Google Gemini client."""
        if not GEMINI_AVAILABLE or not genai:
            logger.warning("Google Generative AI library not available. Chatbot will use mock responses.")
            self.client = None
            self.model = None
        elif not settings.gemini_api_key:
            logger.warning("Gemini API key not configured. Chatbot will use mock responses.")
            self.client = None
            self.model = None
        else:
            try:
                genai.configure(api_key=settings.gemini_api_key)
                self.client = genai
                # Try to initialize model, fallback to gemini-pro if the specified model fails
                try:
                    self.model = genai.GenerativeModel(settings.gemini_model)
                    # Test if model is available by listing models
                    try:
                        available_models = [m.name for m in genai.list_models()]
                        logger.info(f"Available Gemini models: {available_models[:5]}...")
                        if settings.gemini_model not in available_models and f"models/{settings.gemini_model}" not in available_models:
                            logger.warning(f"Model {settings.gemini_model} not found in available models, will try on first use")
                    except Exception as e:
                        logger.debug(f"Could not list models: {e}")
                except Exception as e:
                    logger.warning(f"Failed to initialize model {settings.gemini_model}: {e}, will try on first use")
                    self.model = None
            except Exception as e:
                logger.error(f"Failed to configure Gemini: {e}")
                self.client = None
                self.model = None
    
    def get_embedding(self, text: str) -> List[float]:
        """
        Get embedding vector for text using Google Gemini.
        
        Args:
            text: Text to embed
            
        Returns:
            Embedding vector
        """
        if not self.client:
            # Return mock embedding for development
            return [0.0] * 768  # Gemini embeddings are 768-dimensional
        
        if not GEMINI_AVAILABLE or not genai:
            # Return mock embedding for development
            return [0.0] * 768  # Gemini embeddings are 768-dimensional
        
        try:
            result = genai.embed_content(
                model=settings.gemini_embedding_model,
                content=text,
                task_type="retrieval_document"
            )
            return result['embedding']
        except Exception as e:
            logger.error(f"Error getting embedding: {e}")
            raise ValidationError(f"Failed to generate embedding: {str(e)}")
    
    def retrieve_relevant_context(
        self,
        query: str,
        limit: int = 5,
        score_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context from vector database using RAG.
        
        Args:
            query: User query text
            limit: Maximum number of results
            score_threshold: Minimum similarity score
            
        Returns:
            List of relevant context items with scores
        """
        if not VECTOR_DB_AVAILABLE:
            logger.debug("Vector DB not available, returning empty context")
            return []
        
        try:
            vector_db = get_vector_db()
            # Get query embedding
            query_embedding = self.get_embedding(query)
            
            # Search vector database
            results = vector_db.search(
                query_vector=query_embedding,
                limit=limit,
                score_threshold=score_threshold
            )
            
            return results
        except Exception as e:
            logger.debug(f"Vector DB search failed: {e}, returning empty context")
            return []
    
    def generate_response(
        self,
        query: str,
        context_items: List[Dict[str, Any]],
        user_context: Optional[Dict] = None
    ) -> Dict[str, Any]:
        """
        Generate AI response using Google Gemini LLM with retrieved context.
        
        Args:
            query: User query
            context_items: Retrieved context from vector DB
            user_context: Additional context from user session
            
        Returns:
            Response with text, sources, and confidence score
        """
        # Check if API key is configured
        if not settings.gemini_api_key or not self.client or not self.model:
            # Return mock response for development
            logger.info("No Gemini API key configured, returning mock response")
            return {
                "response_text": "This is a mock response. Please configure GEMINI_API_KEY environment variable for real responses. Get a free API key at https://makersuite.google.com/app/apikey",
                "source_references": [],
                "confidence_score": 0.5
            }
        
        try:
            # Build context from retrieved items
            context_text = ""
            source_references = []
            
            for item in context_items:
                if item.get("payload"):
                    payload = item["payload"]
                    if "content" in payload:
                        context_text += f"\n\n{payload.get('content', '')}"
                    if "url" in payload:
                        source_references.append(payload["url"])
                    elif "section" in payload:
                        source_references.append(payload["section"])
            
            # Add user context if available
            if user_context:
                if "current_page_url" in user_context:
                    context_text += f"\n\nUser is currently viewing: {user_context['current_page_url']}"
                if "selected_text" in user_context:
                    context_text += f"\n\nUser selected text: {user_context['selected_text']}"
            
            # Build prompt for Gemini
            prompt = f"""You are a helpful AI assistant for a Physical AI & Humanoid Robotics textbook. 
Your role is to answer questions about the textbook content accurately and clearly.
Use the provided context from the textbook to answer questions. If the context doesn't contain 
enough information, say so honestly. Always cite sources when possible.

Context from textbook:
{context_text}

User question: {query}

Please provide a clear, accurate answer based on the context above."""
            
            # Call Gemini API
            try:
                response = self.model.generate_content(
                    prompt,
                    generation_config=genai.types.GenerationConfig(
                        temperature=0.7,
                        max_output_tokens=1000,
                    )
                )
                response_text = response.text
            except Exception as model_error:
                # If API call fails, return mock response instead of raising error
                error_msg = str(model_error)
                logger.warning(f"Gemini API call failed: {error_msg[:200]}")
                
                # Return helpful mock response
                return {
                    "response_text": f"I apologize, but I'm having trouble connecting to the AI service right now. This is a mock response.\n\nTo get real AI responses:\n1. Get a free Gemini API key at https://makersuite.google.com/app/apikey\n2. Add it to backend/.env as: GEMINI_API_KEY=your-key-here\n3. Restart the backend server\n\nError: {error_msg[:150]}",
                    "source_references": source_references,
                    "confidence_score": 0.3
                }
            
            # Calculate confidence score based on context quality
            confidence_score = min(1.0, len(context_items) / 5.0) if context_items else 0.3
            if context_items:
                avg_score = sum(item.get("score", 0) for item in context_items) / len(context_items)
                confidence_score = (confidence_score + avg_score) / 2
            
            return {
                "response_text": response_text,
                "source_references": source_references,
                "confidence_score": min(1.0, confidence_score)
            }
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            raise ValidationError(f"Failed to generate response: {str(e)}")
    
    def process_query(
        self,
        db: Session,
        user_id: int,
        query_text: str,
        context: Optional[Dict] = None
    ) -> Dict[str, Any]:
        """
        Process a chatbot query end-to-end: retrieve context, generate response, and save to DB.
        
        Args:
            db: Database session
            user_id: User ID
            query_text: User query
            context: Optional user context
            
        Returns:
            Complete response with query and response data
        """
        try:
            # Try to save query to database (optional for development)
            query_id = None
            try:
                if hasattr(db, 'add'):  # Check if it's a real database session
                    chatbot_query = ChatbotQuery(
                        user_id=user_id,
                        query_text=query_text,
                        context=context or {}
                    )
                    db.add(chatbot_query)
                    db.commit()
                    db.refresh(chatbot_query)
                    query_id = chatbot_query.id
                    logger.info(f"Saved chatbot query {query_id} to database")
            except Exception as db_error:
                logger.debug(f"Database save skipped: {db_error}, continuing without DB")
            
            # Retrieve relevant context
            context_items = self.retrieve_relevant_context(query_text)
            
            # Generate response
            response_data = self.generate_response(
                query_text,
                context_items,
                context
            )
            
            # Try to save response to database (optional)
            try:
                if SQLALCHEMY_AVAILABLE and ChatbotResponse and hasattr(db, 'add') and query_id:  # Check if it's a real database session
                    chatbot_response = ChatbotResponse(
                        query_id=query_id,
                        response_text=response_data["response_text"],
                        source_references=response_data["source_references"],
                        confidence_score=response_data["confidence_score"]
                    )
                    db.add(chatbot_response)
                    db.commit()
                    logger.info(f"Saved chatbot response to database")
            except Exception as db_error:
                logger.debug(f"Database response save skipped: {db_error}")
            
            return {
                "query_id": query_id or 0,
                "response_text": response_data["response_text"],
                "source_references": response_data["source_references"],
                "confidence_score": response_data["confidence_score"]
            }
        except Exception as e:
            logger.error(f"Error processing query: {e}")
            if hasattr(db, 'rollback'):
                try:
                    db.rollback()
                except:
                    pass
            raise


# Global chatbot service instance
chatbot_service = ChatbotService()

