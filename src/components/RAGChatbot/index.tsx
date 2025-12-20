import React, { useState, useRef, useEffect, useCallback } from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

// Types for the RAG chatbot
interface Message {
  id: string;
  role: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: Date;
  sources?: DocumentSource[];
  isLoading?: boolean;
}

interface DocumentSource {
  title: string;
  path: string;
  snippet: string;
  relevanceScore: number;
}

interface DocumentChunk {
  id: string;
  content: string;
  title: string;
  path: string;
  embedding?: number[];
}

// Simple vector similarity using cosine similarity
function cosineSimilarity(a: number[], b: number[]): number {
  if (a.length !== b.length) return 0;
  let dotProduct = 0;
  let normA = 0;
  let normB = 0;
  for (let i = 0; i < a.length; i++) {
    dotProduct += a[i] * b[i];
    normA += a[i] * a[i];
    normB += b[i] * b[i];
  }
  return dotProduct / (Math.sqrt(normA) * Math.sqrt(normB));
}

// Simple text embedding using TF-IDF-like approach (client-side)
function simpleEmbed(text: string, vocabulary: string[]): number[] {
  const words = text.toLowerCase().split(/\s+/);
  const wordFreq: Record<string, number> = {};
  words.forEach(word => {
    wordFreq[word] = (wordFreq[word] || 0) + 1;
  });
  return vocabulary.map(vocabWord => wordFreq[vocabWord] || 0);
}

// Book content knowledge base - paths match actual Docusaurus routes
const BOOK_KNOWLEDGE: DocumentChunk[] = [
  {
    id: '1',
    title: 'Introduction to Physical AI',
    path: '/docs/01-robotic-nervous-system/01-introduction-physical-ai',
    content: `Physical AI describes computational systems that possess physical form, gather real-world information through sensors, and influence their surroundings via actuators. These aren't just algorithms running on servers; they're embodied agents that must navigate the complexities of physics, handle uncertainty, and operate safely in dynamic environments. Physical AI represents the next evolutionary leap in intelligent systems, combining perception, cognition, action, and adaptation in a continuous loop.`
  },
  {
    id: '2',
    title: 'ROS 2 Architecture',
    path: '/docs/01-robotic-nervous-system/02-ros2-architecture',
    content: `ROS 2 (Robot Operating System 2) is the industry-standard framework for robotics development. It uses a node-based architecture where nodes communicate through topics (publish-subscribe), services (request-response), and actions (long-running tasks with feedback). ROS 2 is built on DDS (Data Distribution Service) middleware for reliable, real-time communication. Key concepts include nodes, topics, services, actions, parameters, and the computational graph.`
  },
  {
    id: '3',
    title: 'ROS 2 Python Development',
    path: '/docs/01-robotic-nervous-system/03-ros2-python-development',
    content: `Developing ROS 2 applications in Python uses the rclpy library. You create nodes by extending the Node class, use create_publisher() and create_subscription() for topics, create_service() and create_client() for services. Timers are created with create_timer(). The main loop uses rclpy.spin() to process callbacks. Package structure includes setup.py, package.xml, and resource files.`
  },
  {
    id: '4',
    title: 'URDF Robot Modeling',
    path: '/docs/01-robotic-nervous-system/04-urdf-robot-modeling',
    content: `URDF (Unified Robot Description Format) is an XML format for describing robot models. It defines links (rigid bodies) and joints (connections between links). Joint types include revolute, prismatic, continuous, fixed, floating, and planar. Each link can have visual, collision, and inertial properties. URDF is used for visualization in RViz2 and simulation in Gazebo.`
  },
  {
    id: '5',
    title: 'Gazebo Simulation',
    path: '/docs/02-digital-twin/05-gazebo-fundamentals',
    content: `Gazebo is the primary robot simulator in the ROS ecosystem. It provides physics simulation, sensor simulation, and 3D visualization. You can create custom worlds with SDF (Simulation Description Format), spawn URDF robots, and interact with simulations through ROS 2 topics and services. Gazebo supports multiple physics engines including ODE, Bullet, and DART.`
  },
  {
    id: '6',
    title: 'Physics Simulation',
    path: '/docs/02-digital-twin/06-physics-simulation',
    content: `Physics simulation in robotics involves rigid body dynamics, contact forces, friction, and collision detection. Key properties include mass, inertia tensor, center of mass, friction coefficients, and restitution. Accurate physics simulation is crucial for sim-to-real transfer. Sensor simulation includes cameras, LiDAR, IMU, and force/torque sensors.`
  },
  {
    id: '7',
    title: 'Unity Integration',
    path: '/docs/02-digital-twin/07-unity-integration',
    content: `Unity can be integrated with ROS 2 using the ROS-TCP-Connector package. This enables high-fidelity visualization and advanced simulation scenarios. Unity provides realistic rendering, physics simulation with PhysX, and support for VR/AR applications. The Unity Robotics Hub provides tools for URDF import and ROS message handling.`
  },
  {
    id: '8',
    title: 'NVIDIA Isaac Sim',
    path: '/docs/03-ai-robot-brain/08-nvidia-isaac-sim',
    content: `NVIDIA Isaac Sim is a robotics simulation platform built on Omniverse. It provides photorealistic rendering, accurate physics with PhysX 5, and synthetic data generation for AI training. Isaac Sim supports URDF/SDF import, domain randomization, and seamless ROS 2 integration. It's ideal for training perception models and testing autonomous systems.`
  },
  {
    id: '9',
    title: 'Isaac ROS Perception',
    path: '/docs/03-ai-robot-brain/09-isaac-ros-perception',
    content: `Isaac ROS provides GPU-accelerated perception packages for ROS 2. Key components include Visual SLAM (cuVSLAM) for localization and mapping, object detection with YOLO and other models, semantic segmentation, depth estimation, and AprilTag detection. These packages leverage NVIDIA GPUs for real-time performance.`
  },
  {
    id: '10',
    title: 'Nav2 Path Planning',
    path: '/docs/03-ai-robot-brain/10-nav2-path-planning',
    content: `Nav2 is ROS 2's navigation stack for autonomous mobile robots. It includes global planners (NavFn, Smac), local planners (DWB, TEB), costmaps for obstacle representation, behavior trees for navigation logic, and recovery behaviors. Configuration involves setting up costmap layers, planner parameters, and controller settings.`
  },
  {
    id: '11',
    title: 'Voice-to-Action',
    path: '/docs/04-vision-language-action/11-voice-to-action',
    content: `Voice-to-action systems convert speech to robot commands. OpenAI Whisper provides accurate speech-to-text transcription. The pipeline involves audio capture, speech recognition, natural language understanding, and command generation. ROS 2 nodes can subscribe to audio topics and publish text commands for further processing.`
  },
  {
    id: '12',
    title: 'LLM Cognitive Planning',
    path: '/docs/04-vision-language-action/12-llm-cognitive-planning',
    content: `Large Language Models (LLMs) can be used for high-level cognitive planning in robotics. They interpret natural language instructions, generate action sequences, and adapt to dynamic environments. Prompt engineering is crucial for reliable robot control. LLM outputs are converted to ROS 2 action goals or service calls.`
  },
  {
    id: '13',
    title: 'Capstone: Autonomous Humanoid',
    path: '/docs/04-vision-language-action/13-capstone-autonomous-humanoid',
    content: `The capstone project integrates all course concepts: voice command recognition with Whisper, LLM-based cognitive planning, perception with Isaac ROS, navigation with Nav2, and robot control through ROS 2. The goal is a voice-controlled autonomous humanoid that can understand and execute complex multi-step tasks.`
  },
  {
    id: '14',
    title: 'Hardware Requirements',
    path: '/docs/appendices/hardware-guide',
    content: `Minimum hardware requirements: Ubuntu 22.04 LTS, 8GB RAM (16GB recommended), quad-core processor, 50GB free disk space. For Isaac Sim and Isaac ROS: NVIDIA GPU (RTX 30 series or newer recommended), 16GB+ VRAM for complex simulations. Optional: microphone for voice input, webcam for vision experiments.`
  },
  {
    id: '15',
    title: 'Module 1: Robotic Nervous System',
    path: '/docs/01-robotic-nervous-system',
    content: `Module 1 covers ROS 2 fundamentals including architecture, Python development, and URDF robot modeling. Setup requires Ubuntu 22.04 LTS, build-essential, curl, git, python3-pip. Install colcon with pip install -U colcon-common-extensions. Configure locales for ROS 2 with locale-gen en_US.UTF-8.`
  },
  {
    id: '16',
    title: 'Module 2: Digital Twin',
    path: '/docs/02-digital-twin',
    content: `Module 2 covers simulation environments including Gazebo fundamentals, physics simulation, and Unity integration. Learn to create digital twins of robots for testing and development.`
  },
  {
    id: '17',
    title: 'Module 3: AI Robot Brain',
    path: '/docs/03-ai-robot-brain',
    content: `Module 3 covers NVIDIA Isaac Sim, Isaac ROS perception, and Nav2 path planning for autonomous navigation. Learn AI-powered perception and navigation.`
  },
  {
    id: '18',
    title: 'Module 4: Vision-Language-Action',
    path: '/docs/04-vision-language-action',
    content: `Module 4 covers voice-to-action systems, LLM cognitive planning, and the capstone autonomous humanoid project. Build voice-controlled robots using AI.`
  }
];

// Build vocabulary from all documents
const buildVocabulary = (docs: DocumentChunk[]): string[] => {
  const allWords = new Set<string>();
  docs.forEach(doc => {
    doc.content.toLowerCase().split(/\s+/).forEach(word => {
      if (word.length > 2) allWords.add(word);
    });
  });
  return Array.from(allWords);
};

const VOCABULARY = buildVocabulary(BOOK_KNOWLEDGE);

// Pre-compute embeddings for all documents
BOOK_KNOWLEDGE.forEach(doc => {
  doc.embedding = simpleEmbed(doc.content, VOCABULARY);
});

// RAG retrieval function
function retrieveRelevantDocs(query: string, topK: number = 3): DocumentSource[] {
  const queryEmbedding = simpleEmbed(query, VOCABULARY);
  
  const scored = BOOK_KNOWLEDGE.map(doc => ({
    title: doc.title,
    path: doc.path,
    snippet: doc.content.substring(0, 200) + '...',
    relevanceScore: cosineSimilarity(queryEmbedding, doc.embedding || [])
  }));
  
  return scored
    .sort((a, b) => b.relevanceScore - a.relevanceScore)
    .slice(0, topK)
    .filter(doc => doc.relevanceScore > 0.1);
}

// Generate response based on retrieved context
function generateResponse(query: string, sources: DocumentSource[]): string {
  const queryLower = query.toLowerCase();
  
  // Check for greetings
  if (queryLower.match(/^(hi|hello|hey|greetings)/)) {
    return "Hello! 👋 I'm your Physical AI & Robotics learning assistant. I can help you with:\n\n• **ROS 2** - Architecture, Python development, nodes, topics, services\n• **Robot Modeling** - URDF, links, joints, visualization\n• **Simulation** - Gazebo, Unity, NVIDIA Isaac Sim\n• **AI Integration** - Perception, navigation, SLAM\n• **Voice Control** - Whisper, LLM planning, VLA systems\n\nWhat would you like to learn about?";
  }
  
  // Check for help requests
  if (queryLower.match(/^(help|what can you do|capabilities)/)) {
    return "I'm a RAG-powered assistant for the Physical AI & Humanoid Robotics book. Here's what I can help with:\n\n**📚 Learning Topics:**\n• ROS 2 fundamentals and Python development\n• Robot modeling with URDF\n• Simulation with Gazebo, Unity, Isaac Sim\n• AI perception and navigation\n• Voice-to-action systems\n\n**🔍 Features:**\n• Answer questions about book content\n• Provide code examples and explanations\n• Link to relevant documentation\n• Explain robotics concepts\n\nJust ask me anything about robotics!";
  }

  if (sources.length === 0) {
    return "I couldn't find specific information about that in the book content. Could you try rephrasing your question? I'm best at answering questions about:\n\n• ROS 2 and robot programming\n• URDF and robot modeling\n• Gazebo and simulation\n• NVIDIA Isaac platform\n• Navigation and perception\n• Voice control and LLMs in robotics";
  }

  // Build context-aware response
  let response = "";
  const topSource = sources[0];
  
  // Generate response based on query intent
  if (queryLower.includes('what is') || queryLower.includes('explain') || queryLower.includes('define')) {
    response = `Based on the book content:\n\n${topSource.snippet}\n\n`;
  } else if (queryLower.includes('how to') || queryLower.includes('how do')) {
    response = `Here's what the book covers about this:\n\n${topSource.snippet}\n\n`;
  } else if (queryLower.includes('example') || queryLower.includes('code')) {
    response = `From the relevant section:\n\n${topSource.snippet}\n\nFor detailed code examples, check the linked documentation.\n\n`;
  } else {
    response = `${topSource.snippet}\n\n`;
  }

  // Add source references with proper base URL
  response += "**📖 Related Documentation:**\n";
  sources.forEach((source, idx) => {
    response += `${idx + 1}. [[${source.title}]](${source.path}) (${Math.round(source.relevanceScore * 100)}% relevant)\n`;
  });

  return response;
}

// Main RAG Chatbot Component
export default function RAGChatbot(): React.ReactElement {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '0',
      role: 'assistant',
      content: "👋 Hi! I'm your Physical AI & Robotics learning assistant. Ask me anything about ROS 2, robot simulation, URDF, navigation, or any topic from the book!",
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages, scrollToBottom]);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isTyping) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue.trim(),
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsTyping(true);

    // Simulate processing delay for better UX
    await new Promise(resolve => setTimeout(resolve, 500 + Math.random() * 1000));

    // RAG: Retrieve relevant documents
    const sources = retrieveRelevantDocs(userMessage.content);
    
    // Generate response
    const responseContent = generateResponse(userMessage.content, sources);

    const assistantMessage: Message = {
      id: (Date.now() + 1).toString(),
      role: 'assistant',
      content: responseContent,
      timestamp: new Date(),
      sources: sources.length > 0 ? sources : undefined
    };

    setMessages(prev => [...prev, assistantMessage]);
    setIsTyping(false);
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const clearChat = () => {
    setMessages([
      {
        id: '0',
        role: 'assistant',
        content: "👋 Chat cleared! How can I help you learn about Physical AI & Robotics?",
        timestamp: new Date()
      }
    ]);
  };

  // Helper to render message content with proper links
  const renderMessageContent = (content: string) => {
    return content.split('\n').map((line, i) => {
      // Check for link pattern [[title]](path)
      const linkMatch = line.match(/^\d+\. \[\[([^\]]+)\]\]\(([^)]+)\)/);
      if (linkMatch) {
        const [, title, path] = linkMatch;
        const relevanceMatch = line.match(/\((\d+)% relevant\)/);
        const relevance = relevanceMatch ? relevanceMatch[1] : '';
        return (
          <React.Fragment key={i}>
            <span className={styles.sourceLink}>
              {line.match(/^\d+/)?.[0]}. <Link to={path}>{title}</Link> ({relevance}% relevant)
            </span>
            <br />
          </React.Fragment>
        );
      }
      
      // Bold text
      if (line.startsWith('**') && line.endsWith('**')) {
        return (
          <React.Fragment key={i}>
            <strong>{line.slice(2, -2)}</strong>
            <br />
          </React.Fragment>
        );
      }
      
      // Bullet points
      if (line.startsWith('• ')) {
        return (
          <React.Fragment key={i}>
            <span className={styles.bulletPoint}>{line}</span>
            <br />
          </React.Fragment>
        );
      }
      
      // Regular text
      return (
        <React.Fragment key={i}>
          {line}
          {i < content.split('\n').length - 1 && <br />}
        </React.Fragment>
      );
    });
  };

  const suggestedQuestions = [
    "What is Physical AI?",
    "How do ROS 2 nodes communicate?",
    "Explain URDF robot modeling",
    "What is Nav2 used for?",
    "How does Isaac Sim work?"
  ];

  return (
    <>
      {/* Chat Toggle Button */}
      <button
        className={`${styles.chatToggle} ${isOpen ? styles.chatToggleOpen : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat assistant'}
      >
        {isOpen ? (
          <span className={styles.closeIcon}>✕</span>
        ) : (
          <span className={styles.chatIcon}>💬</span>
        )}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerInfo}>
              <span className={styles.headerIcon}>🤖</span>
              <div>
                <h3 className={styles.headerTitle}>AI Learning Assistant</h3>
                <span className={styles.headerStatus}>
                  <span className={styles.statusDot}></span>
                  RAG-Powered
                </span>
              </div>
            </div>
            <button className={styles.clearButton} onClick={clearChat} title="Clear chat">
              🗑️
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${styles[message.role]}`}
              >
                <div className={styles.messageContent}>
                  {message.role === 'assistant' && (
                    <span className={styles.messageAvatar}>🤖</span>
                  )}
                  <div className={styles.messageText}>
                    {renderMessageContent(message.content)}
                  </div>
                  {message.role === 'user' && (
                    <span className={styles.messageAvatar}>👤</span>
                  )}
                </div>
                <span className={styles.messageTime}>
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </span>
              </div>
            ))}
            
            {isTyping && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageContent}>
                  <span className={styles.messageAvatar}>🤖</span>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            
            <div ref={messagesEndRef} />
          </div>

          {/* Suggested Questions */}
          {messages.length <= 2 && (
            <div className={styles.suggestions}>
              <span className={styles.suggestionsLabel}>Try asking:</span>
              <div className={styles.suggestionButtons}>
                {suggestedQuestions.map((q, i) => (
                  <button
                    key={i}
                    className={styles.suggestionButton}
                    onClick={() => {
                      setInputValue(q);
                      inputRef.current?.focus();
                    }}
                  >
                    {q}
                  </button>
                ))}
              </div>
            </div>
          )}

          {/* Input Area */}
          <div className={styles.inputArea}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask about robotics, ROS 2, simulation..."
              className={styles.input}
              disabled={isTyping}
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputValue.trim() || isTyping}
              className={styles.sendButton}
              aria-label="Send message"
            >
              ➤
            </button>
          </div>
        </div>
      )}
    </>
  );
}
