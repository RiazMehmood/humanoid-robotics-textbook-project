"""Content translation service for multilingual support."""
from typing import Dict, Optional, Any
from src.utils.logger import logger

# Optional Google Generative AI import for translation
try:
    import google.generativeai as genai
    from src.config import settings
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False
    genai = None
    settings = None


class TranslationService:
    """Service for translating content to different languages."""
    
    def __init__(self):
        """Initialize the translation service."""
        if GEMINI_AVAILABLE and genai and settings and settings.gemini_api_key:
            try:
                genai.configure(api_key=settings.gemini_api_key)
                self.client = genai
                self.model = genai.GenerativeModel(settings.gemini_model or "gemini-2.5-flash")
                logger.info("TranslationService initialized with Gemini")
            except Exception as e:
                logger.warning(f"Failed to initialize Gemini for translation: {e}")
                self.client = None
                self.model = None
        else:
            logger.warning("TranslationService initialized without Gemini (API key not configured)")
            self.client = None
            self.model = None
    
    def translate_content(
        self,
        content: str,
        source_language: str = "en",
        target_language: str = "es"
    ) -> Dict[str, Any]:
        """
        Translate content from source language to target language.
        
        Args:
            content: Text content to translate
            source_language: Source language code (e.g., "en", "es")
            target_language: Target language code (e.g., "en", "es")
            
        Returns:
            Dictionary with translated content and metadata
        """
        # If source and target are the same, return original
        if source_language == target_language:
            return {
                "translatedText": content,
                "sourceLanguage": source_language,
                "targetLanguage": target_language,
                "method": "no_translation_needed"
            }
        
        # Try to use Gemini for translation if available
        if self.client and self.model:
            try:
                prompt = f"""Translate the following text from {source_language} to {target_language}.
Preserve the meaning, tone, and technical terminology accurately.
Return only the translated text, no explanations.

Text to translate:
{content}"""
                
                response = self.model.generate_content(
                    prompt,
                    generation_config=genai.types.GenerationConfig(
                        temperature=0.3,
                        max_output_tokens=2000,
                    )
                )
                
                translated_text = response.text.strip()
                
                return {
                    "translatedText": translated_text,
                    "sourceLanguage": source_language,
                    "targetLanguage": target_language,
                    "method": "gemini_ai"
                }
            except Exception as e:
                logger.error(f"Error translating with Gemini: {e}")
                # Fall through to mock translation
        
        # Mock translation for development/testing
        logger.info(f"Using mock translation from {source_language} to {target_language}")
        return {
            "translatedText": f"[TRANSLATED TO {target_language.upper()}] {content}",
            "sourceLanguage": source_language,
            "targetLanguage": target_language,
            "method": "mock"
        }
    
    def translate_markdown_content(
        self,
        markdown_content: str,
        source_language: str = "en",
        target_language: str = "es"
    ) -> Dict[str, Any]:
        """
        Translate markdown content while preserving markdown structure.
        
        Args:
            markdown_content: Markdown formatted content
            source_language: Source language code
            target_language: Target language code
            
        Returns:
            Dictionary with translated markdown and metadata
        """
        # If source and target are the same, return original
        if source_language == target_language:
            return {
                "translatedText": markdown_content,
                "sourceLanguage": source_language,
                "targetLanguage": target_language,
                "method": "no_translation_needed"
            }
        
        # Try to use Gemini for translation if available
        if self.client and self.model:
            try:
                prompt = f"""Translate the following markdown content from {source_language} to {target_language}.
Preserve all markdown formatting, code blocks, links, and structure exactly.
Only translate the text content, not the markdown syntax.

Markdown to translate:
{markdown_content}"""
                
                response = self.model.generate_content(
                    prompt,
                    generation_config=genai.types.GenerationConfig(
                        temperature=0.3,
                        max_output_tokens=4000,
                    )
                )
                
                translated_markdown = response.text.strip()
                
                return {
                    "translatedText": translated_markdown,
                    "sourceLanguage": source_language,
                    "targetLanguage": target_language,
                    "method": "gemini_ai"
                }
            except Exception as e:
                logger.error(f"Error translating markdown with Gemini: {e}")
                # Fall through to mock translation
        
        # Mock translation for development/testing
        logger.info(f"Using mock translation for markdown from {source_language} to {target_language}")
        return {
            "translatedText": f"# [TRANSLATED TO {target_language.upper()}]\n\n{markdown_content}",
            "sourceLanguage": source_language,
            "targetLanguage": target_language,
            "method": "mock"
        }


# Global translation service instance
translation_service = TranslationService()


