#ifndef __CS123XMLSCENEPARSER__
#define __CS123XMLSCENEPARSER__

#include "CS123ISceneParser.h"
#include "CS123SceneData.h"

#include <vector>
#include <map>

#include <QtXml>

struct ImageParameter {
    std::string caustic_img;
    Eigen::Vector2f img_size;
    float img_rotate = 0;
    Eigen::Vector3f img_center;
    Eigen::Vector3f img_normal;

    int lightIndexBegin = -1, lightIndexEnd = -1;
};

/**
 * @class CS123XmlSceneParser
 *
 * This class parses the scene graph specified by the CS123 Xml file format.
 *
 * The parser is designed to replace the TinyXML parser that was in turn designed to replace the
 * Flex/Yacc/Bison parser.
 */
class CS123XmlSceneParser : public CS123ISceneParser {

public:
    // Create a parser, passing it the scene file.
    CS123XmlSceneParser(const std::string& filename);
    CS123XmlSceneParser(const std::string& filename, int _cnt);

    // Clean up all data for the scene
    virtual ~CS123XmlSceneParser();

    // Parse the scene.  Returns false if scene is invalid.
    virtual bool parse();

    virtual bool getGlobalData(CS123SceneGlobalData& data) const;

    virtual bool getCameraData(CS123SceneCameraData& data) const;

    virtual CS123SceneNode* getRootNode() const;

    virtual int getNumLights() const;

    // Returns the ith light data
    virtual bool getLightData(int i, CS123SceneLightData& data) const;

    virtual bool getImageParameters(std::vector<std::shared_ptr<ImageParameter>>& imageParameters) const;


private:
    // The filename should be contained within this parser implementation.
    // If you want to parse a new file, instantiate a different parser.
    bool parseGlobalData(const QDomElement &globaldata);
    bool parseCameraData(const QDomElement &cameradata);
    bool parseLightData(const QDomElement &lightdata);
    bool parseObjectData(const QDomElement &object);
    bool parseTransBlock(const QDomElement &transblock, CS123SceneNode* node);
    bool parsePrimitive(const QDomElement &prim, CS123SceneNode* node);
    bool parseStylized(const QDomElement &stylized);
    bool parseParameters(const QDomElement &params);

    std::string file_name;
    mutable std::map<std::string, CS123SceneNode*> m_objects;
    CS123SceneCameraData m_cameraData;
    std::vector<CS123SceneLightData*> m_lights;
    CS123SceneGlobalData m_globalData;
    std::vector<CS123SceneNode*> m_nodes;
    std::vector<std::shared_ptr<ImageParameter>> imageParameters;

    int cnt = -1;
};

#endif

